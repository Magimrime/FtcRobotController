package ftcsim;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.function.Supplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * One simulated match: the clock, the Driver Station (INIT, button presses, START, STOP), the
 * robot's devices in a HardwareMap, the world they act on, and the record of what happened.
 * The simulated clock only moves when the OpMode does something that takes time on a real
 * Control Hub (hub reads and writes, IMU reads, telemetry, sleep) and the physics runs in steps
 * of at most 1 ms in between, so the robot code sees time pass exactly as it would on the robot.
 */
public class SimContext {
    private static final ThreadLocal<SimContext> CURRENT = new ThreadLocal<>();

    public static SimContext get() {
        SimContext c = CURRENT.get();
        if (c == null) throw new IllegalStateException("no simulation is running on this thread");
        return c;
    }

    public static SimContext getOrNull() {
        return CURRENT.get();
    }

    static void set(SimContext c) {
        if (c == null) CURRENT.remove();
        else CURRENT.set(c);
    }

    /** Thrown into the OpMode when it ignores STOP for too long (the SDK would kill it too). */
    public static class OpModeKilled extends Error {
        public OpModeKilled(String why) {
            super(why);
        }
    }

    public final Scenario scenario;
    public final Random rng;
    public final World world;
    public final CameraModel camera;
    public final HardwareMap hardwareMap = new HardwareMap();
    public final SimTelemetry telemetry;
    public final Gamepad gamepad1 = new Gamepad(), gamepad2 = new Gamepad();
    public final List<String> events = new ArrayList<>();
    public final List<String> deviceNotes = new ArrayList<>();
    public List<String> initScreen = new ArrayList<>();

    private double t = 0;
    private double stopAt;
    private boolean stopRequested = false, opModeDone = false, stopLogged = false;
    private String stopReason = "";
    private final List<double[]> pressTimes = new ArrayList<>();
    private final List<String> pressButtons = new ArrayList<>();
    private int nextPress = 0;
    private final long wallDeadline;
    private long tickCount = 0;
    Supplier<double[]> odometry = () -> null;  // the OpMode's own idea of where it is ("our side"), or null

    // screen event tracking
    private final Map<String, String> lastValue = new LinkedHashMap<>();
    private final Map<String, String> lastShape = new LinkedHashMap<>();
    private final Set<String> seenNotes = new HashSet<>();
    double doneAt = -1;  // match time the OpMode's screen first said "done"

    // devices by role (null if not in the configuration)
    public final SimMotor[] wheels = new SimMotor[4];
    public SimMotor intake, flywheel;
    public SimCRServo leftFeeder, rightFeeder, siphon;
    public SimIMU imu;
    public Limelight3A limelight;
    public SimVisionPortal portal;  // set when the OpMode builds a VisionPortal

    public SimContext(Scenario scenario) {
        this.scenario = scenario;
        this.rng = new Random(scenario.seed * 104729L + 3);
        set(this);
        this.world = new World(this);
        this.camera = new CameraModel(this);
        this.telemetry = new SimTelemetry(this);
        this.stopAt = scenario.initTime + scenario.matchTime;
        this.wallDeadline = System.nanoTime() + 60_000_000_000L;
        buildHardware();
        world.placeRobot();
        planPresses();
    }

    // ================================================================ clock

    public double now() {
        return t;
    }

    public long nanos() {
        return (long) (t * 1e9);
    }

    /** Seconds since START (negative in INIT). */
    public double matchTime() {
        return t - scenario.initTime;
    }

    public boolean isStarted() {
        return t >= scenario.initTime;
    }

    public boolean isStopRequested() {
        return stopRequested || t >= stopAt;
    }

    /** After STOP (or after the OpMode returned) the SDK cuts power to every motor and servo. */
    public boolean robotDisabled() {
        return opModeDone || t >= stopAt;
    }

    public void requestStop(String why) {
        if (!stopRequested) {
            stopRequested = true;
            stopReason = why;
            stopAt = Math.min(stopAt, t);
        }
    }

    public void sleep(double seconds) {
        tick(seconds);
    }

    /** The OpMode spent dt seconds: run the world forward that long. */
    public void tick(double dt) {
        if (scenario.loopJitter > 0 && dt > 0 && rng.nextDouble() < scenario.loopJitter) {
            dt += scenario.loopJitterMs / 1000.0;
        }
        if (portal != null && portal.processingNow()) {
            dt *= scenario.webCpuSlow;  // the tag finder shares the Control Hub's CPU
        }
        advanceTo(t + dt);
        if ((++tickCount & 1023) == 0 && System.nanoTime() > wallDeadline) {
            throw new OpModeKilled("the OpMode ran for 60 s of real time (stuck in a loop that never checks opModeIsActive?)");
        }
        if (t > stopAt + scenario.stopGrace) {
            throw new OpModeKilled(String.format(Locale.US, "the OpMode kept running %.1f s after STOP", scenario.stopGrace));
        }
    }

    /** Runs the physics (and the Driver Station script) to time t1. */
    void advanceTo(double t1) {
        while (t < t1) {
            double step = Math.min(0.001, t1 - t);
            if (nextPress < pressTimes.size() && pressTimes.get(nextPress)[0] < t + step) {
                step = Math.max(1e-6, pressTimes.get(nextPress)[0] - t);
            }
            boolean wasStarted = isStarted();
            world.step(step, t);
            t += step;
            if (!wasStarted && isStarted()) {
                initScreen = new ArrayList<>(telemetry.lastFrame());
                event("START");
            }
            if (!stopLogged && t >= stopAt - 1e-9) {
                stopLogged = true;
                event("STOP (end of AUTO)" + (stopReason.isEmpty() ? "" : ": " + stopReason));
            }
            while (nextPress < pressTimes.size() && pressTimes.get(nextPress)[0] <= t + 1e-9) {
                double[] p = pressTimes.get(nextPress);
                String b = pressButtons.get(nextPress);
                Gamepad g = p[1] == 1 ? gamepad1 : gamepad2;
                if (p[2] > 0) {
                    g.press(b);
                    event("driver presses " + b + " on gamepad " + (int) p[1]);
                } else {
                    g.release(b);
                }
                nextPress++;
            }
        }
    }

    /** The OpMode returned (or crashed): power off, then let the world run to the end of AUTO and a bit. */
    void finishMatch(double extraAfterEnd) {
        opModeDone = true;
        double end = scenario.initTime + scenario.matchTime + extraAfterEnd;
        if (t < end) advanceTo(end);
    }

    // ================================================================ Driver Station script

    private void planPresses() {
        List<String> list = new ArrayList<>();
        String op = scenario.opmode;
        if (op.equals("Auto4") || op.equals("Auto5")) {
            if (scenario.blue()) list.add("y");
            if (scenario.start.equals("far")) list.add("dpad_up");
            if (scenario.pick.equals("near")) list.add("x");
            if (scenario.pick.equals("far")) {
                list.add("x");
                list.add("x");
            }
        } else if (op.equals("Auto3")) {
            if (scenario.path.equals("red")) list.add("x");
        }
        for (String p : scenario.presses.split(",")) {
            if (!p.trim().isEmpty()) list.add(p.trim());
        }
        // presses start 2 s into INIT (after the camera setup), 0.4 s apart, held 0.15 s
        double at = Math.min(2.0, scenario.initTime * 0.5);
        for (String b : list) {
            pressTimes.add(new double[] {at, 1, 1});
            pressButtons.add(b);
            pressTimes.add(new double[] {at + 0.15, 1, 0});
            pressButtons.add(b);
            at += 0.4;
        }
        if (at > scenario.initTime - 0.5 && !list.isEmpty()) {
            throw new IllegalArgumentException("initTime too short for the INIT button presses");
        }
    }

    // ================================================================ hardware

    private void buildHardware() {
        Scenario s = scenario;
        LynxModule control = new LynxModule("Control Hub");
        LynxModule expansion = new LynxModule("Expansion Hub 2");
        // Which names the robot configuration has. Default: the names the code asks for,
        // on the hubs / ports of TeamCode/hardware-config/biobuzz.xml.
        Map<String, String[]> config = new LinkedHashMap<>();  // name -> {type, hub, port}
        if (s.config.isEmpty()) {
            config.put("mainmotorintake", new String[] {"motor", "EH", "0"});
            config.put("topleft", new String[] {"motor", "EH", "1"});
            config.put("topright", new String[] {"motor", "EH", "2"});
            config.put("rightfeeder", new String[] {"servo", "EH", "0"});
            config.put("servosiphon", new String[] {"servo", "EH", "1"});
            config.put("turret", new String[] {"motor", "CH", "0"});
            config.put("bottomleft", new String[] {"motor", "CH", "2"});
            config.put("bottomright", new String[] {"motor", "CH", "3"});
            config.put("leftfeeder", new String[] {"servo", "CH", "0"});
            config.put("imu", new String[] {"imu", "CH", "0"});
            if (s.camera) config.put("limelight", new String[] {"limelight", "", ""});
            if (s.camera && s.webcam) config.put("Webcam 1", new String[] {"webcam", "", ""});
        } else {
            config = readConfig(s.config);
            if (!s.camera) {
                config.remove("limelight");
                config.values().removeIf(v -> v[0].equals("webcam"));
            }
        }
        hardwareMap.put(null, control);
        hardwareMap.put(null, expansion);
        for (Map.Entry<String, String[]> e : config.entrySet()) {
            String name = e.getKey();
            String[] v = e.getValue();
            LynxModule hub = v[1].equals("EH") ? expansion : control;
            int port = v[2].isEmpty() ? 0 : Integer.parseInt(v[2]);
            switch (v[0]) {
                case "motor": {
                    SimMotor m;
                    switch (name) {
                        // TRUE wiring: the code's directions (FL, BL, BR REVERSE) drive every wheel forward
                        case "topleft": m = wheels[0] = new SimMotor(name, hub, port, 537.7, -1, -1); break;
                        case "topright": m = wheels[1] = new SimMotor(name, hub, port, 537.7, 1, 1); break;
                        case "bottomleft": m = wheels[2] = new SimMotor(name, hub, port, 537.7, -1, -1); break;
                        case "bottomright":
                            m = wheels[3] = new SimMotor(name, hub, port, 537.7, -1, s.brEncoderBackward ? 1 : -1);
                            break;
                        case "mainmotorintake": m = intake = new SimMotor(name, hub, port, 537.7, 1, 1); break;
                        case "turret":
                        case "flywheel": m = flywheel = new SimMotor(name, hub, port, 28, 1, 1); break;
                        default: m = new SimMotor(name, hub, port, 537.7, 1, 1); break;
                    }
                    hardwareMap.put(name, m);
                    break;
                }
                case "servo": {
                    SimCRServo sv;
                    switch (name) {
                        case "leftfeeder": sv = leftFeeder = new SimCRServo(name, port, 1); break;
                        case "rightfeeder": sv = rightFeeder = new SimCRServo(name, port, -1); break;
                        case "servosiphon": sv = siphon = new SimCRServo(name, port, -1); break;
                        default: sv = new SimCRServo(name, port, 1); break;
                    }
                    hardwareMap.put(name, sv);
                    break;
                }
                case "imu":
                    imu = new SimIMU(world, s, new Random(s.seed * 31 + 7));
                    hardwareMap.put(name, imu);
                    break;
                case "limelight":
                    limelight = new Limelight3A(camera, name);
                    hardwareMap.put(name, limelight);
                    break;
                case "webcam":
                    hardwareMap.put(name, new SimWebcam(name));
                    break;
                default:
                    break;
            }
        }
        for (int i = 0; i < 4; i++) {
            if (wheels[i] != null && s.deadEncoder == i) wheels[i].encoderDead = true;
        }
        String[] wheelNames = {"topleft", "topright", "bottomleft", "bottomright"};
        for (int i = 0; i < 4; i++) {
            if (wheels[i] == null) deviceNotes.add("configuration has no " + wheelNames[i]);
        }
        if (flywheel == null) deviceNotes.add("configuration has no turret / flywheel motor");
    }

    /** Device names, types and hubs from a Robot Configuration XML. */
    static Map<String, String[]> readConfig(String path) {
        Map<String, String[]> out = new LinkedHashMap<>();
        try {
            String xml = new String(Files.readAllBytes(Paths.get(path)), "UTF-8");
            String hub = "CH";
            Matcher m = Pattern.compile("<(\\w+)\\s+([^>]*?)/?>").matcher(xml);
            while (m.find()) {
                String tag = m.group(1), attrs = m.group(2);
                String name = attr(attrs, "name"), port = attr(attrs, "port");
                if (tag.equals("LynxModule")) {
                    hub = "173".equals(port) || "Control Hub".equals(name) ? "CH" : "EH";
                } else if (tag.contains("Motor")) {
                    out.put(name, new String[] {"motor", hub, port == null ? "0" : port});
                } else if (tag.contains("ContinuousRotationServo") || tag.equals("Servo")) {
                    out.put(name, new String[] {"servo", hub, port == null ? "0" : port});
                } else if (tag.contains("Imu") || tag.contains("IMU")) {
                    out.put(name, new String[] {"imu", hub, "0"});
                } else if (tag.equals("Webcam")) {
                    out.put(name, new String[] {"webcam", "", ""});
                } else if (tag.equals("EthernetDevice")) {
                    out.put(name, new String[] {"limelight", "", ""});
                }
            }
        } catch (java.io.IOException e) {
            throw new IllegalArgumentException("can't read robot configuration " + path + ": " + e);
        }
        return out;
    }

    static String attr(String attrs, String key) {
        Matcher m = Pattern.compile(key + "=\"([^\"]*)\"").matcher(attrs);
        return m.find() ? m.group(1) : null;
    }

    // ================================================================ record

    /** What the OpMode's screen says it is doing now ("Step", else "Doing"). */
    String currentStep() {
        String v = lastValue.get("Step");
        return v != null ? v : lastValue.getOrDefault("Doing", "");
    }

    public void event(String text) {
        events.add(String.format(Locale.US, "%6.2f  %s", matchTime(), text));
    }

    /** Every telemetry frame: log step changes and new notes the OpMode puts on its screen. */
    void onScreen(List<String> frame) {
        for (String line : frame) {
            int sep = line.indexOf(" : ");
            if (sep > 0 && sep < 12) {
                String cap = line.substring(0, sep);
                if (cap.equals("Step") || cap.equals("Doing")) {
                    String v = line.substring(sep + 3);
                    String shape = v.replaceAll("[-+]?\\d+(\\.\\d+)?", "#");  // "0.12 of 1.00 turns" counts as one step
                    if (!shape.equals(lastShape.get(cap))) {
                        lastShape.put(cap, shape);
                        lastValue.put(cap, v);
                        if (cap.equals("Step") || !lastValue.containsKey("Step")) {
                            event(cap.toUpperCase(Locale.US) + ": " + v);
                        } else {
                            event("  doing: " + v);
                        }
                        if (cap.equals("Doing") && v.equals("done") && doneAt < 0) doneAt = matchTime();
                    }
                }
            } else if (line.startsWith("  ") && !line.startsWith("   ") && line.trim().length() > 0 && isStarted()) {
                String n = line.trim();
                if (n.matches("^\\d+\\.\\d s .*")) continue;  // Auto4's list of steps done (already logged)
                if (seenNotes.add(n)) event("  screen: " + n);
            }
        }
    }
}
