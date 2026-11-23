package ftcsim;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;

/**
 * Everything one simulated match depends on. Every field can be set from the command line as
 * name=value (arrays as 1,2,3). The noise preset (none / typical / harsh) draws the random parts
 * from the seed FIRST; key=value overrides are applied after it, so they always win.
 *
 * TRUE values describe the real robot and field; the robot code keeps its own guesses (its
 * constants), and the difference between the two is exactly what the simulator tests.
 */
public class Scenario implements Cloneable {

    // ---------------------------------------------------------------- what is run
    public String opmode = "Auto4";      // class in TeamCode: Auto4 or Auto3
    public String alliance = "red";      // red / blue
    public String start = "near";        // near / far (Auto4's two start spots; Auto3 always starts near)
    public String up = "near";           // which of OUR CELLs really points up at the start: near / far
    public String pick = "camera";       // Auto4's X pick in INIT: camera / near / far
    public String path = "green";        // Auto3's X pick in INIT: green / red
    public String presses = "";          // extra INIT button presses, e.g. "x,x" (comma list)
    public String noise = "typical";     // none / typical / harsh
    public long seed = 1;
    public String name = "";             // label for files; made up if empty
    public String config = "";           // robot configuration XML to take device names from ("" = the names the code uses)

    // ---------------------------------------------------------------- Driver Station timeline
    public double initTime = 4.0;        // seconds of INIT before START (camera watches, picks are pressed)
    public double matchTime = 30.0;      // AUTO length
    public double stopGrace = 2.0;       // seconds the OpMode may keep running after STOP before it is killed

    // ---------------------------------------------------------------- loop timing (seconds of simulated time per call)
    public double costLoopCheck = 0.00005;  // opModeIsActive() etc.
    public double costBulkRead = 0.0022;    // a REV hub bulk read
    public double costHubCommand = 0.0012;  // one hub command (setPower that changes, a read without bulk caching)
    public double costImuRead = 0.0018;     // BHI260AP yaw/pitch/roll read
    public double costTelemetry = 0.0008;   // building and sending a telemetry frame
    public double costCameraRead = 0.00002; // Limelight getLatestResult (cached in the SDK)
    public double costHttp = 0.03;          // Limelight status / pipeline switch / upload (blocking HTTP)
    public double loopJitter = 0.0;         // chance per call of a slow loop (GC pause, Wi-Fi)
    public double loopJitterMs = 20;

    // ---------------------------------------------------------------- the robot's body and drive (TRUE)
    public double halfLen = 8.5, halfWid = 8.5;  // inches from the middle to the front / to a side
    public double wheelK = (350 + 300) / 2.0 / 25.4;  // lx + ly, inches (Auto4's TURN_RADIUS)
    public double wheelFreeSpeed = 54;      // in/s of a wheel at full power, 12 V, under the robot's weight
    public double tauBody = 0.12;           // s: how fast the robot follows the wheel commands when gripping
    public double tauWheelSlip = 0.04;      // s: how fast a slipping wheel spins up / down
    public double aMaxF = 1e9, aMaxS = 1e9, alphaMax = 1e9;  // traction limits: in/s^2 forward, sideways; rad/s^2
    public double kSlip = 0;                // slipping wheel speed: 0 = rolls with the robot, 1 = spins at full command
    public double strafeEff = 0.9;          // sideways inches per inch the encoders count (Auto4's STRAFE_FACTOR guess)
    public double turnEff = 1.0;            // turning: real / wheel-counted
    public double[] wheelGain = {1, 1, 1, 1};   // FL FR BL BR motor strength
    public double[] diamErr = {0, 0, 0, 0};     // wheel diameter error, fraction (0.01 = 1 % bigger)
    public boolean brEncoderBackward = true;    // BR power wires swapped (memory 2026-10-05): its encoder counts backward
    public int deadEncoder = -1;                 // 0-3: that wheel's encoder never counts
    public double wallSquare = 0.05;        // rad of squaring per inch pushed into a wall at an angle

    // ---------------------------------------------------------------- IMU (TRUE)
    public double imuBias = 0;     // deg/s drift
    public double imuScale = 0;    // fraction: 0.005 = reads 0.5 % too much turn
    public double imuNoise = 0.005;  // deg noise per sample (never 0: a real IMU never repeats exactly)
    public double imuWalk = 0;     // deg/sqrt(s) random walk
    public double imuFreezeAt = -1;  // match second at which the IMU stops updating (-1 never)

    // ---------------------------------------------------------------- battery
    public double batteryV0 = 12.6;   // volts at START
    public double batteryR = 0;       // ohms internal resistance (sag = R x current)
    public double batteryDrain = 0;   // volts lost per second

    // ---------------------------------------------------------------- launcher, siphon, HIVE
    public double siphonSecPerTurn = 1.5;  // TRUE seconds per siphon turn at power 0.3 (code guesses 1.5)
    public double flywheelTau = 0.25, flywheelAccel = 2500, shotDip = 0.07;
    public double shotRangeRef = 53.5, shotRpmRef = 2800;  // at 2800 RPM a ball drops into a CELL 53.5 in. from the robot's middle
    public double shotSigmaLat = 0.3, shotSigmaRange = 0.4;  // inches of spread per ball
    public double shotMiss = 0;          // chance a well-aimed ball bounces out anyway
    public String missBalls = "";        // these balls (1 = first ball shot in the match, comma list) bounce out of the CELL
    public String noTip = "";            // these volleys (1 = first, comma list) go in but do NOT tip the HIVE (too light / stuck)
    public int gardenCount = 4;          // POLLEN really lying in our GARDEN (a partner may have taken some); the ones nearest the corner stay
    public String missPickups = "";      // GARDEN balls (1 = nearest our alliance wall, comma list) the intake fails to grab
    public double intakeMiss = 0;        // chance the intake fails to grab each GARDEN ball it passes over
    public double launcherYawErr = 0;    // degrees the launcher shoots off the robot's back
    public double shotFlight = 0.5;      // seconds in the air
    public double cellOpenHalfX = 6.0, cellOpenHalfY = 4.5;  // GUESS: half the CELL opening (along the tag row, across it)
    public int tipBalls = 3;             // GUESS: POLLEN into the up CELL that tip the HIVE
    public double tipDelay = 0.5;        // seconds from the tipping ball to the HIVE resting the other way
    public double partnerTip = -1;       // match second at which our PARTNER tips our HIVE (-1 never)
    public String hive = "practice";     // practice (default): as Auto5's run logs saw Jin's practice HIVE: tags 29.5 / 8.5 in. high
                                         // (up / down CELL), faces tilting 45 deg; guess: 34 / 28 in., tilting 17.5 deg (first sim)
    public double upH = 34, downH = 28;  // GUESS: height of the tags under an up / down CELL
    public double hiveArm = 10;          // CELL middles are this far either side of the HIVE pivot
    public double tagTilt = Double.NaN;  // degrees the tag faces tilt with the HIVE (NaN = from upH, downH, hiveArm)
    public String theirUp = "random";    // the other alliance's HIVE: audience / scoring / random
    public boolean hiveLegs = true;      // GUESS: the HIVE frame stands on 4 legs at its corners (49.46 x 38.95 in. frame)
    public double legHalf = 1.0;         // GUESS: half the width of a leg (2 in. square)

    // ---------------------------------------------------------------- start placement error ("our side" inches / degrees)
    public double startErrX = 0, startErrY = 0, startErrH = 0;

    // ---------------------------------------------------------------- camera (TRUE mount and behavior)
    public boolean camera = true;        // false = no "limelight" in the configuration
    public boolean camNoFrames = false;  // true = configured but no pictures ever arrive (cable)
    public double camBack = 8, camHeight = 8, camPitch = 15, camYaw = 0, camRight = 0;  // TRUE mount (code: 8, 8, 15)
    public double camHfov = 54.5, camVfov = 42;
    public double camFps1 = 45, camFps2 = 55;   // pictures/s at tag downscale 1 / 2+
    public double camLatency = 0.03;            // s from picture to result on the hub
    public double camDownscale = 1;             // stored "fiducial_resdiv" of the tag pipeline
    public String camPipe1Type = "pipe_fiducial";  // what pipeline 1 is before the code touches it
    public double camSigmaPx = 0.05;            // corner noise in pixels (sets 3D noise)
    public double camScaleErr = 0;              // per-run distance scale error (calibration), fraction
    public double camMisread = 0;               // chance per tag per picture of a bad 3D pose (10-30 % range error)
    public double camDropRate = 0, camDropLen = 0.5;  // dropouts per second, and how long one lasts
    public double camOcclude = 0;               // chance a tag is hidden (HIVE frame bar) for the whole run
    public double camMinPx = 7, camGoodPx = 13; // tag's short side in pixels (after downscale): never / always found
    public double camMaxAngle = 80, camGoodAngle = 70;  // degrees off straight-on: never / always found
    public double camPDetect = 0.98;
    // the USB webcam (Auto5: Logitech C270 "Webcam 1", same TRUE mount as cam*; the Control Hub finds the tags)
    public boolean webcam = true;               // "Webcam 1" in the configuration (camera=false removes it too)
    public double webHfov = 45.2, webVfov = 34.7;  // C270: 55 deg diagonal at 640 x 480
    public double webFps = 30;                  // pictures a second from the webcam
    public double webOpenTime = 1.5;            // seconds from build() to STREAMING
    public double webProcMs1 = 85, webProcMs2 = 45, webProcMs3 = 32;  // GUESS: ms the Control Hub needs per picture at decimation 1 / 2 / 3
    public double webCpuSlow = 1.3;             // the Control Hub's loop is this much slower while the tag finder runs

    // ---------------------------------------------------------------- knocks
    public double bumpAt = -1, bumpX = 0, bumpY = 0, bumpH = 0;  // match second, "our side" inches / degrees

    // ---------------------------------------------------------------- scoring zones ("our side", GUESS until test T10)
    public double lzX0 = 0, lzX1 = 24, lzY0 = 120, lzY1 = 144;

    // ================================================================ building scenarios

    public boolean blue() {
        return alliance.equalsIgnoreCase("blue");
    }

    public double tiltDeg() {
        if (!Double.isNaN(tagTilt)) return tagTilt;
        return Math.toDegrees(Math.asin(Math.max(-1, Math.min(1, (upH - downH) / (2 * hiveArm)))));
    }

    /** A copy with these key=value settings (noise preset resolved with the seed, then overrides). */
    public static Scenario of(Map<String, String> kv) {
        Scenario s = new Scenario();
        // the identity fields first, so the noise preset knows the seed
        for (String k : new String[] {"opmode", "alliance", "start", "up", "pick", "path", "noise", "seed", "name", "config", "presses", "hive"}) {
            if (kv.containsKey(k)) s.set(k, kv.get(k));
        }
        s.applyNoise();
        if (s.hive.equals("practice")) {
            s.upH = 29.5;
            s.downH = 8.5;
            s.tagTilt = 45;
        } else if (!s.hive.equals("guess")) {
            throw new IllegalArgumentException("hive must be guess or practice, not " + s.hive);
        }
        for (Map.Entry<String, String> e : kv.entrySet()) {
            s.set(e.getKey(), e.getValue());
        }
        if (s.name.isEmpty()) {
            s.name = s.defaultName();
        }
        return s;
    }

    public String defaultName() {
        StringBuilder b = new StringBuilder();
        b.append(opmode).append('-').append(alliance).append('-');
        if (opmode.equals("Auto3")) {
            b.append("near-up").append(up).append('-').append(path);
        } else {
            b.append(start).append("-up").append(up).append('-').append(pick);
        }
        b.append('-').append(noise).append(hive.equals("practice") ? "" : "-" + hive + "HIVE").append("-s").append(seed);
        return b.toString();
    }

    /** Draws the random TRUE values of the noise preset from the seed. */
    void applyNoise() {
        Random r = new Random(seed * 7919L + 17);
        switch (noise) {
            case "none":
                break;
            case "typical":
                strafeEff = u(r, 0.82, 0.90);
                turnEff = u(r, 0.94, 1.0);
                for (int i = 0; i < 4; i++) {
                    wheelGain[i] = u(r, 0.97, 1.03);
                    diamErr[i] = u(r, -0.007, 0.007);
                }
                aMaxF = 230;
                aMaxS = 170;
                alphaMax = 20;
                kSlip = 0.25;
                imuBias = u(r, -0.03, 0.03);
                imuScale = u(r, -0.003, 0.003);
                imuNoise = 0.02;
                imuWalk = 0.01;
                batteryV0 = u(r, 12.4, 13.0);
                batteryR = 0.05;
                batteryDrain = 0.005;
                siphonSecPerTurn = u(r, 1.35, 1.65);
                shotSigmaLat = 1.0;
                shotSigmaRange = 1.5;
                shotMiss = 0.05;
                launcherYawErr = u(r, -0.5, 0.5);
                startErrX = r.nextGaussian() * 0.5;
                startErrY = Math.abs(r.nextGaussian()) * 0.2;
                startErrH = r.nextGaussian() * 1.0;
                camBack = 8 + u(r, -0.5, 0.5);
                camHeight = 8 + u(r, -0.5, 0.5);
                camPitch = 15 + u(r, -1.5, 1.5);
                camYaw = u(r, -1, 1);
                camSigmaPx = 0.4;
                camScaleErr = r.nextGaussian() * 0.015;
                camMisread = 0.01;
                camDropRate = 0.02;
                camDropLen = 0.5;
                camOcclude = 0.1;
                break;
            case "harsh":
                strafeEff = u(r, 0.75, 0.85);
                turnEff = u(r, 0.88, 1.0);
                for (int i = 0; i < 4; i++) {
                    wheelGain[i] = u(r, 0.94, 1.06);
                    diamErr[i] = u(r, -0.015, 0.015);
                }
                aMaxF = 170;
                aMaxS = 120;
                alphaMax = 14;
                kSlip = 0.4;
                imuBias = u(r, -0.08, 0.08);
                imuScale = u(r, -0.008, 0.008);
                imuNoise = 0.05;
                imuWalk = 0.03;
                batteryV0 = u(r, 11.9, 12.3);
                batteryR = 0.08;
                batteryDrain = 0.01;
                siphonSecPerTurn = u(r, 1.25, 1.8);
                shotSigmaLat = 1.8;
                shotSigmaRange = 2.5;
                shotMiss = 0.12;
                launcherYawErr = u(r, -1.5, 1.5);
                startErrX = r.nextGaussian() * 1.5;
                startErrY = Math.abs(r.nextGaussian()) * 0.5;
                startErrH = r.nextGaussian() * 2.5;
                camBack = 8 + u(r, -1, 1);
                camHeight = 8 + u(r, -1, 1);
                camPitch = 15 + u(r, -3, 3);
                camYaw = u(r, -2, 2);
                camSigmaPx = 0.8;
                camScaleErr = r.nextGaussian() * 0.03;
                camMisread = 0.04;
                camDropRate = 0.08;
                camDropLen = 1.0;
                camOcclude = 0.2;
                break;
            default:
                throw new IllegalArgumentException("noise must be none, typical or harsh, not " + noise);
        }
    }

    static double u(Random r, double lo, double hi) {
        return lo + (hi - lo) * r.nextDouble();
    }

    /** Sets one field by name from text. */
    public void set(String key, String value) {
        try {
            Field f = Scenario.class.getField(key);
            Class<?> t = f.getType();
            if (t == double.class) f.setDouble(this, Double.parseDouble(value));
            else if (t == int.class) f.setInt(this, Integer.parseInt(value));
            else if (t == long.class) f.setLong(this, Long.parseLong(value));
            else if (t == boolean.class) f.setBoolean(this, value.equals("1") || value.equalsIgnoreCase("true") || value.equalsIgnoreCase("on"));
            else if (t == String.class) f.set(this, value);
            else if (t == double[].class) {
                String[] parts = value.split(",");
                double[] a = new double[parts.length];
                for (int i = 0; i < a.length; i++) a[i] = Double.parseDouble(parts[i].trim());
                f.set(this, a);
            } else throw new IllegalArgumentException("can't set " + key);
        } catch (NoSuchFieldException e) {
            throw new IllegalArgumentException("unknown setting \"" + key + "\" (see sim/README.md for the list)");
        } catch (IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    /** All settings as key=value lines, for the run log. */
    public List<String> describe() {
        List<String> lines = new ArrayList<>();
        for (Field f : Scenario.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers())) continue;
            try {
                Object v = f.get(this);
                String text;
                if (v instanceof double[]) {
                    StringBuilder b = new StringBuilder();
                    for (double d : (double[]) v) b.append(b.length() > 0 ? "," : "").append(fmt(d));
                    text = b.toString();
                } else if (v instanceof Double) {
                    text = fmt((Double) v);
                } else {
                    text = String.valueOf(v);
                }
                lines.add(f.getName() + "=" + text);
            } catch (IllegalAccessException e) {
                throw new RuntimeException(e);
            }
        }
        return lines;
    }

    static String fmt(double d) {
        if (d == Math.rint(d) && Math.abs(d) < 1e7) return String.valueOf((long) d);
        return String.format(Locale.US, "%.4g", d);
    }

    public static Map<String, String> parse(List<String> args) {
        Map<String, String> kv = new LinkedHashMap<>();
        for (String a : args) {
            int eq = a.indexOf('=');
            if (eq <= 0) throw new IllegalArgumentException("expected key=value, got " + a);
            kv.put(a.substring(0, eq).trim(), a.substring(eq + 1).trim());
        }
        return kv;
    }
}
