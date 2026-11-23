package ftcsim;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.TimeUnit;

/**
 * The Logitech C270 on the Control Hub. It opens over webOpenTime seconds, then streams webFps
 * pictures a second. An enabled AprilTag processor takes webProcMs per picture on the Control Hub
 * (longer with less decimation) and skips pictures that arrive while it is busy, so a look gets
 * about 20 results a second at decimation 2. While the tag finder runs, the Control Hub's own loop
 * is slower too (webCpuSlow, see SimContext.tick). Tags come from CameraModel.observe with the
 * webcam's field of view.
 */
public class SimVisionPortal extends VisionPortal {
    final SimContext ctx;
    final Scenario s;
    final CameraName camera;
    final Size size;
    final List<VisionProcessor> processors;
    final Map<VisionProcessor, Boolean> enabled = new IdentityHashMap<>();
    final double builtAt;
    boolean closed = false, streaming = true;
    double nextFrame = 0, busyUntil = 0, dropUntil = -1, lastStep = 0;
    int processed = 0;

    private static class Pending {
        double readyAt;
        List<CameraModel.TagSeen> seen;
        long frameNanos;
        SimAprilTagProcessor to;
    }

    private final List<Pending> pending = new ArrayList<>();
    private final Controls controls = new Controls();

    public SimVisionPortal(CameraName camera, Size size, List<VisionProcessor> processors) {
        ctx = SimContext.get();
        s = ctx.scenario;
        this.camera = camera;
        this.size = size;
        this.processors = new ArrayList<>(processors);
        for (VisionProcessor p : processors) enabled.put(p, true);
        ctx.tick(0.05);
        builtAt = ctx.now();
        nextFrame = builtAt + s.webOpenTime;
        ctx.portal = this;
    }

    double procSeconds(SimAprilTagProcessor p) {
        double d = p.decimation;
        double ms = d <= 1 ? s.webProcMs1 : d <= 2 ? s.webProcMs2 : s.webProcMs3;
        return ms / 1000.0;
    }

    /** True while an enabled tag finder is chewing on pictures (the Control Hub runs slower). */
    boolean processingNow() {
        if (closed || !streaming || getCameraState() != CameraState.STREAMING) return false;
        for (VisionProcessor p : processors) {
            if (p instanceof SimAprilTagProcessor && enabled.get(p)) return true;
        }
        return false;
    }

    int latestTagCount() {
        for (VisionProcessor p : processors) {
            if (p instanceof SimAprilTagProcessor && enabled.get(p)) return ((SimAprilTagProcessor) p).latestCount();
        }
        return 0;
    }

    void step(double t) {
        double dt = t - lastStep;
        lastStep = t;
        if (closed || !streaming || s.camNoFrames || t < builtAt + s.webOpenTime) return;
        if (t >= dropUntil && s.camDropRate > 0 && ctx.rng.nextDouble() < s.camDropRate * dt) {
            dropUntil = t + s.camDropLen;
            if (ctx.isStarted()) ctx.event(String.format(Locale.US, "webcam DROPOUT for %.1f s (no pictures)", s.camDropLen));
        }
        if (t >= nextFrame) {
            nextFrame += 1.0 / s.webFps;
            if (nextFrame < t) nextFrame = t + 1.0 / s.webFps;
            if (t >= dropUntil) {
                for (VisionProcessor p : processors) {
                    if (!(p instanceof SimAprilTagProcessor) || !enabled.get(p) || t < busyUntil) continue;
                    SimAprilTagProcessor tp = (SimAprilTagProcessor) p;
                    Pending pd = new Pending();
                    pd.seen = ctx.camera.observe(s.webHfov, s.webVfov, tp.decimation, s.camMinPx, s.camGoodPx);
                    pd.readyAt = t + procSeconds(tp);
                    pd.frameNanos = (long) (t * 1e9);
                    pd.to = tp;
                    busyUntil = pd.readyAt;
                    pending.add(pd);
                }
            }
        }
        for (int i = 0; i < pending.size(); i++) {
            Pending pd = pending.get(i);
            if (pd.readyAt <= t) {
                if (enabled.get(pd.to)) {
                    pd.to.deliver(pd.seen, pd.frameNanos);
                    processed++;
                }
                pending.remove(i--);
            }
        }
    }

    @Override
    public void setProcessorEnabled(VisionProcessor processor, boolean on) {
        ctx.tick(0.0005);
        enabled.put(processor, on);
    }

    @Override
    public boolean getProcessorEnabled(VisionProcessor processor) {
        Boolean b = enabled.get(processor);
        return b != null && b;
    }

    @Override
    public CameraState getCameraState() {
        double t = ctx.now();
        if (closed) return CameraState.CAMERA_DEVICE_CLOSED;
        if (s.camNoFrames) return t < builtAt + 0.8 ? CameraState.OPENING_CAMERA_DEVICE : CameraState.ERROR;
        if (t < builtAt + 0.8) return CameraState.OPENING_CAMERA_DEVICE;
        if (t < builtAt + s.webOpenTime) return CameraState.STARTING_STREAM;
        return streaming ? CameraState.STREAMING : CameraState.CAMERA_DEVICE_READY;
    }

    @Override
    public void saveNextFrameRaw(String filename) {
        if (ctx.isStarted()) ctx.event("webcam picture saved as " + filename + " (not written by the simulator)");
    }

    @Override
    public void stopStreaming() {
        streaming = false;
    }

    @Override
    public void resumeStreaming() {
        streaming = true;
    }

    @Override
    public void stopLiveView() {
    }

    @Override
    public void resumeLiveView() {
    }

    @Override
    public float getFps() {
        if (getCameraState() != CameraState.STREAMING) return 0;
        double fps = s.webFps;
        for (VisionProcessor p : processors) {
            if (p instanceof SimAprilTagProcessor && enabled.get(p)) fps = Math.min(fps, 1 / procSeconds((SimAprilTagProcessor) p));
        }
        return (float) fps;
    }

    @Override
    @SuppressWarnings("unchecked")
    public <T extends CameraControl> T getCameraControl(Class<T> controlType) {
        if (controlType.isInstance(controls)) return (T) controls;
        return null;
    }

    @Override
    public void setActiveCamera(WebcamName webcamName) {
    }

    @Override
    public WebcamName getActiveCamera() {
        return camera instanceof WebcamName ? (WebcamName) camera : null;
    }

    @Override
    public void close() {
        closed = true;
    }

    /** The C270's exposure and gain (accepted; the simulated picture does not depend on them). */
    static class Controls implements ExposureControl, GainControl {
        Mode mode = Mode.Auto;
        long exposureNs = 10_000_000;
        int gain = 64;

        @Override public Mode getMode() { return mode; }
        @Override public boolean setMode(Mode m) { mode = m; return true; }
        @Override public boolean isModeSupported(Mode m) { return m == Mode.Auto || m == Mode.Manual || m == Mode.AperturePriority; }
        @Override public long getMinExposure(TimeUnit u) { return u.convert(1, TimeUnit.MILLISECONDS); }
        @Override public long getMaxExposure(TimeUnit u) { return u.convert(204, TimeUnit.MILLISECONDS); }
        @Override public long getExposure(TimeUnit u) { return u.convert(exposureNs, TimeUnit.NANOSECONDS); }
        @Override public boolean setExposure(long d, TimeUnit u) { exposureNs = TimeUnit.NANOSECONDS.convert(d, u); return mode == Mode.Manual; }
        @Override public boolean isExposureSupported() { return true; }
        @Override public boolean getAePriority() { return false; }
        @Override public boolean setAePriority(boolean p) { return true; }
        @Override public int getMinGain() { return 0; }
        @Override public int getMaxGain() { return 255; }
        @Override public int getGain() { return gain; }
        @Override public boolean setGain(int g) { gain = Math.max(0, Math.min(255, g)); return true; }
    }
}
