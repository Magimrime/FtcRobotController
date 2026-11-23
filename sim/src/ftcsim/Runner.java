package ftcsim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Supplier;

/** Runs ONE scenario: the real OpMode class from TeamCode against the simulated robot. */
public class Runner {

    public static class Result {
        public Scenario s;
        public boolean pass;
        public final List<String> failReasons = new ArrayList<>();
        public final List<String> notes = new ArrayList<>();
        public int points, tips, partnerTips;
        public boolean leave, park;
        public int midlineFouls, legHits, downVolleys, volleyCount;
        public double midlineMax, legDepthMax, legClearMin, midClearMin;
        public String crash = "";
        public double doneAt = -1;     // match second the OpMode finished (returned or showed "done"); -1 = still busy at 30 s
        public double odoErrMax, odoErrMean, odoErrEnd = Double.NaN, headErrMax;
        public double[] endPose;
        public double wallGap;
        public int frames, tagsFound;
        public List<World.Volley> volleys;
        public List<String> events, initScreen, finalScreen, settings, fouls;
        public List<double[]> trace;
        public List<String> traceStep;
        public List<double[]> balls, garden;
        public double[] gardenTaken, gardenMissed;
        public List<String> robotLog;  // the OpMode's own run log (Auto5's auto5-log-N.csv), if it wrote one
        public String tipTimes = "";
        public double wallSeconds;

        public String verdict() {
            return pass ? "PASS" : "FAIL";
        }

        public String oneLine() {
            return String.format(Locale.US, "%-4s %3d pts  tips %d%s  %s%s  volleys %d  done %s  odo err max %4.1f in  end (%5.1f, %5.1f, %3.0f)  %s",
                    verdict(), points, tips, tips > 0 ? " @" + tipTimes : "", leave ? "LEAVE " : "", park ? "PARK" : "",
                    volleyCount, doneAt < 0 ? "  --  " : String.format(Locale.US, "%5.1fs", doneAt), odoErrMax,
                    endPose == null ? 0 : endPose[0], endPose == null ? 0 : endPose[1], endPose == null ? 0 : endPose[2],
                    pass ? String.join("; ", notes) : String.join("; ", failReasons));
        }
    }

    static final Object ROBOT_LOG_LOCK = new Object();

    /**
     * OpModes that write their own run log on the Control Hub (Auto5: AppUtil.ROBOT_DATA_DIR) write
     * it to one shared folder, so those matches run one at a time and the log is taken along.
     */
    public static Result run(Scenario s) {
        if (!s.opmode.equals("Auto5")) return runOne(s);
        synchronized (ROBOT_LOG_LOCK) {
            java.io.File dir = org.firstinspires.ftc.robotcore.internal.system.AppUtil.ROBOT_DATA_DIR;
            dir.mkdirs();
            java.io.File[] old = dir.listFiles();
            if (old != null) for (java.io.File f : old) f.delete();
            Result r = runOne(s);
            java.io.File[] made = dir.listFiles();
            if (made != null && made.length > 0) {
                try {
                    r.robotLog = Files.readAllLines(made[0].toPath(), StandardCharsets.UTF_8);
                } catch (IOException e) {
                    r.notes.add("could not read the OpMode's run log: " + e.getMessage());
                }
            }
            return r;
        }
    }

    static Result runOne(Scenario s) {
        long wall0 = System.nanoTime();
        SimContext ctx = new SimContext(s);
        Result r = new Result();
        r.s = s;
        try {
            LinearOpMode op;
            try {
                Class<?> cls = Class.forName("org.firstinspires.ftc.teamcode." + s.opmode);
                op = (LinearOpMode) cls.getDeclaredConstructor().newInstance();
            } catch (ReflectiveOperationException e) {
                throw new IllegalArgumentException("no OpMode class " + s.opmode + " compiled into the simulator (see run-sim.ps1)", e);
            }
            op.hardwareMap = ctx.hardwareMap;
            op.telemetry = ctx.telemetry;
            op.gamepad1 = ctx.gamepad1;
            op.gamepad2 = ctx.gamepad2;
            ctx.odometry = odometryReader(op);
            ctx.event("INIT pressed");
            try {
                op.runOpMode();
                if (ctx.isStarted() && ctx.matchTime() < s.matchTime) {
                    ctx.event("OpMode returned");
                    if (ctx.doneAt < 0) ctx.doneAt = ctx.matchTime();
                } else if (!ctx.isStarted()) {
                    r.crash = "the OpMode returned during INIT";
                }
            } catch (SimContext.OpModeKilled e) {
                r.crash = "KILLED: " + e.getMessage();
                ctx.event("!! " + r.crash);
            } catch (Throwable e) {
                ctx.odometry = () -> null;  // the OpMode is gone: nothing to compare with
                StringWriter sw = new StringWriter();
                e.printStackTrace(new PrintWriter(sw));
                String[] lines = sw.toString().split("\\R");
                StringBuilder b = new StringBuilder(e.toString());
                for (int i = 1; i < Math.min(lines.length, 6); i++) b.append(" | ").append(lines[i].trim());
                r.crash = (ctx.isStarted() ? "CRASH in AUTO: " : "CRASH in INIT: ") + b;
                ctx.event("!! " + r.crash);
            }
            r.finalScreen = new ArrayList<>(ctx.telemetry.lastFrame());
            ctx.finishMatch(3.0);  // power off; let balls land and the HIVE settle
        } finally {
            SimContext.set(null);
        }
        score(ctx, r);
        r.wallSeconds = (System.nanoTime() - wall0) / 1e9;
        return r;
    }

    /** Reads (never writes) the OpMode's own odometry fields, as "our side" X, Y, heading. */
    static Supplier<double[]> odometryReader(Object op) {
        Class<?> c = op.getClass();
        try {
            Field fx = c.getDeclaredField("posX"), fy = c.getDeclaredField("posY"), fh = c.getDeclaredField("heading");
            fx.setAccessible(true);
            fy.setAccessible(true);
            fh.setAccessible(true);
            return () -> {
                try {
                    return new double[] {fx.getDouble(op), fy.getDouble(op), fh.getDouble(op)};
                } catch (IllegalAccessException e) {
                    return null;
                }
            };
        } catch (NoSuchFieldException ignored) {
        }
        try {  // Auto3: inches ahead / right of the near start spot, degrees turned left
            Field fa = c.getDeclaredField("posAhead"), fr = c.getDeclaredField("posRight"), fh = c.getDeclaredField("heading");
            fa.setAccessible(true);
            fr.setAccessible(true);
            fh.setAccessible(true);
            double[] st = ftcsim.Field.NEAR_START;
            return () -> {
                try {
                    return new double[] {st[0] - fr.getDouble(op), st[1] - fa.getDouble(op), st[2] + fh.getDouble(op)};
                } catch (IllegalAccessException e) {
                    return null;
                }
            };
        } catch (NoSuchFieldException ignored) {
        }
        return () -> null;
    }

    static void score(SimContext ctx, Result r) {
        World w = ctx.world;
        Scenario s = r.s;
        r.tips = w.ourTips;
        r.partnerTips = w.partnerTips;
        r.tipTimes = String.join(",", w.tipTimes);
        r.leave = w.endPose != null && !w.endTouchingWall;
        r.park = w.endPose != null && w.endInLoadingZone;
        r.points = 20 * r.tips + (r.leave ? 3 : 0) + (r.park ? 5 : 0);
        r.midlineFouls = w.midlineFouls;
        r.midlineMax = w.midlineMax;
        r.legHits = w.legHits;
        r.legDepthMax = w.legDepthMax;
        r.legClearMin = w.legClearMin;
        r.midClearMin = w.midClearMin;
        r.volleys = w.volleys;
        r.volleyCount = w.volleys.size();
        for (World.Volley v : w.volleys) {
            // the robot's decision is the volley: was the CELL it went for up when it started shooting?
            if (!v.targetWasUp) r.downVolleys++;
            else if (v.launchedAtDown > 0) r.notes.add(v.launchedAtDown + " ball(s) of a volley left after the HIVE had already tipped");
        }
        r.doneAt = ctx.doneAt;
        r.odoErrMax = w.odoErrMax;
        r.odoErrMean = w.odoErrN == 0 ? 0 : w.odoErrSum / w.odoErrN;
        r.headErrMax = w.odoHeadErrMax;
        r.endPose = w.endPose;
        if (w.endPose != null && w.endOdo != null) r.odoErrEnd = Math.hypot(w.endOdo[0] - w.endPose[0], w.endOdo[1] - w.endPose[1]);
        r.wallGap = w.endWallGap;
        r.frames = ctx.camera.framesMade;
        r.tagsFound = ctx.camera.tagsFound;
        r.events = ctx.events;
        r.initScreen = ctx.initScreen;
        r.settings = s.describe();
        r.fouls = w.fouls;
        r.trace = w.trace;
        r.traceStep = w.traceStep;
        r.balls = w.balls;
        r.garden = w.garden;
        r.gardenTaken = w.gardenTaken;
        r.gardenMissed = w.gardenMissed;
        int missedPickups = 0, picked = 0;
        for (double[] g : w.garden) {
            if (g[2] == 2) missedPickups++;
            if (g[2] == 1) picked++;
        }
        if (missedPickups > 0 || s.gardenCount < 4) {
            r.notes.add("picked up " + picked + " GARDEN POLLEN" + (missedPickups > 0 ? ", the intake missed " + missedPickups : "")
                    + (s.gardenCount < 4 ? " (only " + s.gardenCount + " were there)" : ""));
        }
        if (r.finalScreen == null) r.finalScreen = new ArrayList<>();

        if (!r.crash.isEmpty()) r.failReasons.add(r.crash);
        if (r.midlineFouls > 0) r.failReasons.add(String.format(Locale.US, "crossed the midline (G402), up to %.1f in.", r.midlineMax));
        if (r.legHits > 0) r.failReasons.add(String.format(Locale.US, "hit a HIVE frame leg (%d time%s, %.1f in. deep)",
                r.legHits, r.legHits > 1 ? "s" : "", r.legDepthMax));
        if (r.downVolleys > 0) r.failReasons.add(r.downVolleys + " volley(s) shot at a DOWN CELL");
        if (r.tips == 0) r.failReasons.add("no TIP");
        r.pass = r.failReasons.isEmpty();
        if (!r.leave) r.notes.add(w.endPose == null ? "no end pose" : "no LEAVE (touching a wall)");
        if (!r.park) r.notes.add("no PARK");
        if (r.doneAt < 0 && r.crash.isEmpty()) r.notes.add("still busy at 30 s");
        if (r.odoErrMax > 6) r.notes.add(String.format(Locale.US, "odometry off by up to %.1f in.", r.odoErrMax));
        for (String d : ctx.deviceNotes) r.notes.add(d);
    }

    // ================================================================ files

    public static void write(Result r, Path dir) throws IOException {
        Files.createDirectories(dir);
        String n = r.s.name;
        List<String> log = new ArrayList<>();
        log.add("SCENARIO " + n);
        log.add("RESULT   " + r.oneLine());
        if (!r.failReasons.isEmpty()) log.add("FAILED   " + String.join("; ", r.failReasons));
        log.add(String.format(Locale.US, "POINTS   %d = %d TIPS x 20 + LEAVE %d + PARK %d   (partner tips: %d)", r.points, r.tips,
                r.leave ? 3 : 0, r.park ? 5 : 0, r.partnerTips));
        log.add(String.format(Locale.US, "ODOMETRY error vs the true pose: max %.1f in., mean %.1f in., at the end %.1f in.; heading max %.1f deg",
                r.odoErrMax, r.odoErrMean, r.odoErrEnd, r.headErrMax));
        log.add(String.format(Locale.US, "CAMERA   %d pictures, %d tag sightings", r.frames, r.tagsFound));
        log.add(String.format(Locale.US, "CLEARANCE closest to the midline %.1f in., closest to a HIVE leg %.1f in. (negative = into it)",
                r.midClearMin, r.legClearMin));
        log.add("");
        log.add("VOLLEYS");
        for (World.Volley v : r.volleys) log.add("  " + v.describe());
        if (r.volleys.isEmpty()) log.add("  none");
        log.add("");
        log.add("RULES");
        for (String f : r.fouls) log.add("  " + f);
        if (r.fouls.isEmpty()) log.add("  no midline crossing, no HIVE leg contact");
        log.add("");
        log.add("EVENTS (match seconds; negative = INIT)");
        for (String e : r.events) log.add("  " + e);
        log.add("");
        log.add("DRIVER STATION SCREEN AT START (end of INIT)");
        for (String l : r.initScreen) log.add("  | " + l);
        log.add("");
        log.add("DRIVER STATION SCREEN AT THE END");
        for (String l : r.finalScreen) log.add("  | " + l);
        log.add("");
        log.add("SETTINGS (TRUE robot / field; the code keeps its own constants)");
        for (String l : r.settings) log.add("  " + l);
        Files.write(dir.resolve(n + ".log"), log, StandardCharsets.UTF_8);

        List<String> csv = new ArrayList<>();
        csv.add(World.TRACE_COLUMNS + ",step");
        for (int i = 0; i < r.trace.size(); i++) {
            double[] row = r.trace.get(i);
            StringBuilder b = new StringBuilder();
            for (double d : row) b.append(Double.isNaN(d) ? "" : String.format(Locale.US, "%.2f", d)).append(',');
            b.append('"').append(r.traceStep.get(i).replace('"', '\'')).append('"');
            csv.add(b.toString());
        }
        Files.write(dir.resolve(n + ".csv"), csv, StandardCharsets.UTF_8);
        Files.write(dir.resolve(n + ".svg"), Svg.draw(r).getBytes(StandardCharsets.UTF_8));
        if (r.robotLog != null) {
            Files.write(dir.resolve(n + ".robotlog.csv"), r.robotLog, StandardCharsets.UTF_8);
        }
    }
}
