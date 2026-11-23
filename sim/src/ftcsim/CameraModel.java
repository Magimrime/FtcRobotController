package ftcsim;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * The Limelight 3A. Pictures at the pipeline's frame rate; each one sees the 16 HIVE tags from the
 * camera's TRUE pose on the robot (which may differ from the code's CAMERA_BACK / HEIGHT / PITCH).
 *
 * HIVE TAGS: 4 in a row under each CELL, facing down, at upH (CELL up) or downH (CELL down). The
 * whole HIVE rocks like a seesaw, so the tag faces tilt by tiltDeg toward the end that is up.
 *
 * A tag is found when it is in the 54.5 x 42 degree picture, faces the camera, and its short side is
 * big enough in pixels after the pipeline's downscale (camMinPx .. camGoodPx), and it is not too
 * steep (camGoodAngle .. camMaxAngle). Its 3D position gets PnP-like noise: mostly along the line of
 * sight, growing with distance squared and steepness, plus a per-run scale error, rare bad poses
 * (misreads), tags hidden behind the frame for the whole run (occlusion) and dropouts (no pictures).
 */
public class CameraModel {
    private final SimContext ctx;
    private final Scenario s;
    private final Random rng;
    private final JSONObject[] pipelines = new JSONObject[10];
    private final String[] applied = new String[10];
    private final boolean[] hidden = new boolean[16];
    private int active = 0;
    private boolean running = false;
    private int pollHz = 100;
    private double nextCapture = 0, dropUntil = -1, lastStep = 0;
    private final double bootMs;

    private static class Pending {
        double arrive;
        LLResult result;
    }

    private final List<Pending> pending = new ArrayList<>();
    private LLResult latest;
    private double latestArrive = -1;
    public int framesMade, tagsFound;

    CameraModel(SimContext ctx) {
        this.ctx = ctx;
        this.s = ctx.scenario;
        this.rng = new Random(s.seed * 7777L + 5);
        this.bootMs = 1000 * (100 + rng.nextInt(1000));
        try {
            for (int i = 0; i < pipelines.length; i++) {
                pipelines[i] = new JSONObject("{\"pipeline_type\":\"pipe_color\",\"desc\":\"slot " + i + "\"}");
            }
            pipelines[1] = new JSONObject("{\"pipeline_type\":\"" + s.camPipe1Type + "\",\"desc\":\"tags\"}");
            pipelines[1].put("fiducial_resdiv", s.camDownscale);
        } catch (JSONException e) {
            throw new RuntimeException(e);
        }
        for (int i = 0; i < pipelines.length; i++) applied[i] = pipelines[i].optString("pipeline_type", "pipe_color");
        for (int i = 0; i < 16; i++) hidden[i] = rng.nextDouble() < s.camOcclude;
    }

    // ================================================================ the SDK side

    public void setRunning(boolean on) {
        running = on;
        if (on) nextCapture = ctx.now();
    }

    public void setPollRateHz(int hz) {
        pollHz = hz;
    }

    public boolean isConnected() {
        return running && !s.camNoFrames;
    }

    public long msSinceLastUpdate() {
        return latest == null ? 100000 : (long) ((ctx.now() - latestArrive) * 1000);
    }

    /** Tags in the newest picture on the hub (0 if none / not on the tag pipeline). */
    public int latestTagCount() {
        return latest == null || ctx.now() - latestArrive > 0.2 ? 0 : latest.getFiducialResults().size();
    }

    public LLResult latestResult() {
        return latest;
    }

    public LLResult lastResultEver() {
        return latest;
    }

    public String activePipelineType() {
        return applied[active];
    }

    public int activePipeline() {
        return active;
    }

    public double fps() {
        return downscale() <= 1 ? s.camFps1 : s.camFps2;
    }

    double downscale() {
        return pipelines[active].optDouble("fiducial_resdiv", s.camDownscale);
    }

    public boolean reloadPipeline() {
        applied[active] = pipelines[active].optString("pipeline_type", "pipe_color");
        return true;
    }

    public boolean switchPipeline(int index) {
        if (index < 0 || index >= pipelines.length) return false;
        active = index;
        applied[index] = pipelines[index].optString("pipeline_type", "pipe_color");
        return true;
    }

    public JSONObject pipelineJson(int index) {
        try {
            return new JSONObject(pipelines[index].toString());
        } catch (JSONException e) {
            return new JSONObject();
        }
    }

    public boolean updatePipeline(JSONObject json, boolean flush) {
        try {
            java.util.Iterator<String> keys = json.keys();
            while (keys.hasNext()) {
                String k = keys.next();
                pipelines[active].put(k, json.opt(k));
            }
            return true;
        } catch (JSONException e) {
            return false;
        }
    }

    public boolean uploadPipeline(String json, Integer index) {
        try {
            int i = index == null ? active : index;
            pipelines[i] = new JSONObject(json);
            return true;
        } catch (JSONException e) {
            return false;
        }
    }

    // ================================================================ pictures

    void step(double t) {
        double dt = t - lastStep;
        lastStep = t;
        if (!running || s.camNoFrames) return;
        if (t >= dropUntil && s.camDropRate > 0 && rng.nextDouble() < s.camDropRate * dt) {
            dropUntil = t + s.camDropLen;
            if (ctx.isStarted()) ctx.event(String.format(Locale.US, "camera DROPOUT for %.1f s (no pictures)", s.camDropLen));
        }
        if (t >= nextCapture) {
            nextCapture += 1.0 / fps();
            if (nextCapture < t) nextCapture = t + 1.0 / fps();
            if (t >= dropUntil) {
                Pending p = new Pending();
                p.result = null;
                p.arrive = t + s.camLatency + rng.nextDouble() / pollHz;
                p.result = capture(t, p.arrive);
                pending.add(p);
                framesMade++;
            }
        }
        for (int i = 0; i < pending.size(); i++) {
            Pending p = pending.get(i);
            if (p.arrive <= t) {
                if (latest == null || p.result.getTimestamp() > latest.getTimestamp()) {
                    latest = p.result;
                    latestArrive = p.arrive;
                }
                pending.remove(i--);
            }
        }
    }

    static double ramp(double v, double lo, double hi) {
        return v <= lo ? 0 : v >= hi ? 1 : (v - lo) / (hi - lo);
    }

    private LLResult capture(double t, double arrive) {
        List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
        String type = applied[active];
        if (type.equals("pipe_fiducial")) {
            for (TagSeen o : observe(s.camHfov, s.camVfov, Math.max(1, downscale()), s.camMinPx, s.camGoodPx)) {
                Pose3D pose = new Pose3D(new Position(DistanceUnit.METER, o.x * 0.0254, o.y * 0.0254, o.z * 0.0254, (long) (t * 1e9)),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, (long) (t * 1e9)));
                tags.add(new LLResultTypes.FiducialResult(o.id, pose, o.ax, -o.ay, o.sizePx * o.sizePx * o.cosO / (640 * 480) * 100, o.u, o.v));
                tagsFound++;
            }
        }
        return new LLResult(bootMs + t * 1000, active, type, tags, 10, 15, (long) (arrive * 1e9));
    }

    /** One HIVE tag the camera found in a picture. */
    public static class TagSeen {
        public int id;                       // 30-45
        public double x, y, z;               // its middle in the camera frame (x right, y down, z out of the lens), inches, with noise
        public double u, v, ax, ay;          // pixel (640 x 480 picture) and degrees off the picture's middle
        public double sizePx, shortPx, cosO; // its size in pixels, its short side after downscale, cos(angle off straight-on)
    }

    /**
     * Which HIVE tags a camera on the robot's back (TRUE mount: camBack, camHeight, camPitch, camYaw)
     * finds in one 640 x 480 picture with this field of view, and where it puts them. Used by the
     * Limelight and by the webcam. downscale = the detector's decimation; minPx / goodPx = the tag's
     * short side in pixels (after downscale) at which it is never / always found.
     */
    List<TagSeen> observe(double hfov, double vfov, double ds, double minPx, double goodPx) {
        List<TagSeen> out = new ArrayList<>();
        World wd = ctx.world;
        double h = Math.toRadians(wd.hDeg);
        double fx = Math.cos(h), fy = Math.sin(h), rx = Math.sin(h), ry = -Math.cos(h);
        double cx = wd.x - s.camBack * fx + s.camRight * rx, cy = wd.y - s.camBack * fy + s.camRight * ry, cz = s.camHeight;
        double psi = Math.toRadians(wd.hDeg + 180 + s.camYaw), p = Math.toRadians(s.camPitch);
        double[] zc = {Math.cos(p) * Math.cos(psi), Math.cos(p) * Math.sin(psi), Math.sin(p)};
        double[] xc = {Math.sin(psi), -Math.cos(psi), 0};
        double[] yc = cross(zc, xc);
        double f = 320 / Math.tan(Math.toRadians(hfov / 2));
        double tilt = Math.toRadians(s.tiltDeg());
        for (int cell = 0; cell < 4; cell++) {
            boolean up = wd.cellUp(cell);
            boolean audUp = wd.audienceUp[Field.hiveOf(cell)];
            double[] n = {0, (audUp ? -1 : 1) * Math.sin(tilt), -Math.cos(tilt)};
            for (int k = 0; k < 4; k++) {
                int id = cell * 4 + k;
                if (hidden[id]) continue;
                double[] tag = {Field.CELL_XY[cell][0] + Field.TAG_PLACE[k], Field.CELL_XY[cell][1], up ? s.upH : s.downH};
                double[] d = {tag[0] - cx, tag[1] - cy, tag[2] - cz};
                double X = dot(d, xc), Y = dot(d, yc), Z = dot(d, zc);
                if (Z < 2) continue;
                double dist = Math.sqrt(dot(d, d));
                double half = Math.toDegrees(Math.atan(Field.TAG_SIZE / 2 / dist));
                double ax = Math.toDegrees(Math.atan2(X, Z)), ay = Math.toDegrees(Math.atan2(Y, Z));
                if (Math.abs(ax) + half > hfov / 2 || Math.abs(ay) + half > vfov / 2) continue;
                double cosO = -dot(n, d) / dist;
                if (cosO <= 0.02) continue;
                double obliq = Math.toDegrees(Math.acos(Math.min(1, cosO)));
                double shortPx = f * Field.TAG_SIZE * cosO / dist / ds;
                double pFind = s.camPDetect * ramp(shortPx, minPx, goodPx) * (1 - ramp(obliq, s.camGoodAngle, s.camMaxAngle));
                if (rng.nextDouble() >= pFind) continue;
                // 3D pose noise: mostly along the line of sight, growing with distance squared and steepness
                double sd = s.camSigmaPx * dist * dist / (f * Field.TAG_SIZE * Math.max(cosO, 0.25));
                double sl = s.camSigmaPx * dist / f + 0.01;
                double[] los = {X / dist, Y / dist, Z / dist};
                double[] e1 = norm(cross(los, new double[] {0, 1, 0}));
                double[] e2 = cross(los, e1);
                double scale = 1 + s.camScaleErr;
                if (rng.nextDouble() < s.camMisread) {
                    scale *= 1 + (rng.nextBoolean() ? 1 : -1) * (0.1 + 0.2 * rng.nextDouble());
                }
                double a = rng.nextGaussian() * sd, b = rng.nextGaussian() * sl, c = rng.nextGaussian() * sl;
                TagSeen o = new TagSeen();
                o.id = 30 + id;
                o.x = X * scale + los[0] * a + e1[0] * b + e2[0] * c;
                o.y = Y * scale + los[1] * a + e1[1] * b + e2[1] * c;
                o.z = Z * scale + los[2] * a + e1[2] * b + e2[2] * c;
                o.u = 320 + f * X / Z;
                o.v = 240 + f * Y / Z;
                o.ax = ax;
                o.ay = ay;
                o.sizePx = f * Field.TAG_SIZE / dist;
                o.shortPx = shortPx;
                o.cosO = cosO;
                out.add(o);
            }
        }
        return out;
    }

    static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    static double[] norm(double[] a) {
        double l = Math.sqrt(dot(a, a));
        return l < 1e-9 ? new double[] {1, 0, 0} : new double[] {a[0] / l, a[1] / l, a[2] / l};
    }
}
