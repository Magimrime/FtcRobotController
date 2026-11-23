package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/*
 * Auto5 Webcam: Auto4 Camera with the LOGITECH C270 WEBCAM instead of the Limelight. Everything
 * else is Auto4's: the same picks, plan, brain, routes, odometry and camera checks. Auto4 stays as it is.
 * The plan behind it is CAMERA_AUTO_PLAN.md (doc/camera-auto-plan), sections 4 and 5.
 *
 * IN AUTO ONLY TIPS SCORE (20 each). So the robot only shoots at the CELL that points UP, and after
 * a volley it looks again to see if the HIVE tipped. Then: collect the GARDEN, and shoot whichever
 * CELL is up now, from the spot right behind it. At the end it parks in the LOADING ZONE if there
 * is time, or at least backs off the wall (LEAVE).
 *
 * BEFORE START (INIT), with either gamepad:
 *   B / Y          alliance: B = RED, Y = BLUE
 *   dpad down      start on the NEAR-WALL spot (red C1, blue D6): Auto3's start
 *   dpad up        start on the FAR-WALL spot  (red C6, blue D1)
 *   X              which CELL is up: CAMERA (the camera decides) -> NEAR -> FAR -> CAMERA ...
 * The screen shows the picks in big letters and draws the field the way the audience sees it,
 * with the robot, both HIVEs and the route it will drive. Check it against the real field.
 * Nothing pressed: RED, NEAR WALL, CAMERA.
 * Close the Camera Stream view on the Driver Station first: while it is open the gamepads don't work.
 *
 * THE ROBOT ALWAYS STARTS with its front (intake) pressed on the wall and its back (camera and
 * launcher) facing our HIVE.
 *
 * WHERE THINGS ARE: every position here is "our side" of the field, the same for both alliances
 * (blue is the red field turned half a turn):
 *   X = inches from OUR alliance wall (red wall for red, blue wall for blue). The middle line is 72.
 *   Y = inches from the NEAR wall, the wall behind our HIVE's near CELL (red: the audience wall,
 *       blue: the wall opposite the audience).
 *   heading = degrees the FRONT points: 0 toward the middle line, 90 toward the far wall,
 *       180 toward our alliance wall, 270 toward the near wall.
 * Our near CELL is at (59, 62), our far CELL at (59, 82). The robot never goes past X = 72 (G402).
 *
 * THE WEBCAM: the Logitech C270, "Webcam 1" in the Robot Configuration (plug it into the Control
 * Hub, Scan, save). It sits where the Limelight was: on the BACK next to the launcher, looking the
 * way it shoots (somewhere else: change CAMERA_BACK, CAMERA_HEIGHT, CAMERA_PITCH). The Control Hub
 * itself finds the AprilTags in its 640x480 picture (the only size the SDK has the C270's lens
 * numbers for, so the distances come out right). That costs the Control Hub a lot of time, so the
 * tag finder only runs in INIT and while the robot stands still for a look; while it drives it is
 * off and odometry gets the Control Hub's full speed. The "Webcam" line shows STREAMING and the
 * pictures a second (expect about 20). No tags from the start spot: TAG_DECIMATION = 1. Picture
 * too dark: raise EXPOSURE_MS or GAIN (EXPOSURE_MS = 0: the webcam's own automatic exposure).
 * The camera answers two questions, by comparing what it sees NOW with what it saw before (see TIP_MOVE):
 *   before a volley   is the CELL straight behind UP? (its tags where an up CELL's tags were)
 *   after a volley    did the HIVE TIP? (that CELL's tags moved or are gone)
 * What it then does (changed after run 1, where the HIVE did not tip but the robot drove off anyway):
 *   did NOT tip           that CELL is still up: collect the GARDEN, back to this same spot, shoot it again
 *   tipped                the other CELL is up: GARDEN, cross under the HIVE, shoot the other CELL
 *   CELL behind is down   don't shoot it: cross to the other CELL (and back again at most once)
 * No answer (no pictures) = it does what the plan says (shoots, and assumes the volley tipped the HIVE).
 * CAMERA_DECIDES = false: the camera only reports on the screen and never changes the plan.
 * The camera never changes the plan on ONE look: "that CELL is down" and "it did NOT tip" are
 * looked at twice, and both looks must agree.
 *
 * ODOMETRY AND CAMERA WORK TOGETHER (Auto4's, tried in a simulator with the Limelight, 2026-11-22,
 * not yet on the robot; the webcam gives the same tag positions, so the same math):
 *   - odometry (wheels + IMU) drives every path; pressing a wall resets Y, the GARDEN corner X and Y
 *   - every look also fixes X from where our CELLs appear (FIX_WEIGHT of the way, never a big jump),
 *     and the heading when both CELLs are seen well
 *   - then, if odometry says it is not straight in front of the CELL, it slides along the wall (realign)
 *   - the aim compares camera and odometry: they agree = camera, they don't = odometry
 *   - it turns around at X 56 and never drives past X 62, so even a turning robot stays off the midline
 * Measure STRAFE_FACTOR first (see the knob): the simulator showed sideways slip is the biggest error.
 *
 * RUN LOG: every run writes /sdcard/FIRST/data/auto5-log-N.csv on the Control Hub (see LOG): the
 * odometry position, target and wheel / intake / siphon commands every loop, and every step, drive
 * and turn command, camera look and note. Plug the Control Hub into the laptop to fetch it.
 *
 * THE CLOCK RULE: before each step it checks that the step still fits in the 30 s (from the
 * T_ estimates below). If not, it skips to the end: park, or LEAVE.
 *
 * BLOCKS: drivePath(spots, endsAtWall)  turnTo(heading)  shootHere()  tipCheck()  collectGarden()
 *         cross()  finish()   The brain that picks the next block is decide().
 */
@Autonomous(name = "Auto5 Webcam", group = "Robot", preselectTeleOp = "TeleOp")
public class Auto5 extends LinearOpMode {

    // ================================================================ THE FIELD (our side, see above)

    static final double ROBOT_HALF = 8.5;  // inches from the robot's middle to its front: middle of a robot pressed on a wall
    static final double ROBOT_SIDE = 8.5;  // inches from the robot's middle to its right side (guess: measure it)
    static final double TURN_X = 56;  // turn around here: the turning robot's corners stay 4 in. off the midline
    static final double MAX_X = 72 - ROBOT_HALF - 1.5;  // never drive the middle past this: the robot stays 1.5 in. off the midline
    // The three start spots: {X, Y, heading}. The robot is pressed on the wall, back to the HIVE.
    static final double[] NEAR_SPOT = {60, ROBOT_HALF, 270};        // red C1, blue D6 (Auto3's start)
    static final double[] FAR_SPOT = {62, 144 - ROBOT_HALF, 90};    // red C6, blue D1 (Auto3's second shot)
    static final double[] SIDE_SPOT = {ROBOT_HALF, 72, 180};        // NOT a start spot (Jin): no button picks it
    static final double FAR_CELL_X = 59;
    static final double[] NEAR_CELL_XY = {59, 62}, FAR_CELL_XY = {FAR_CELL_X, 82};

    // THE ROUTES, as spots {X, Y} in inches. Each list is driven as ONE motion (see drivePath).
    // A spot past a wall is on purpose: the wall stops the robot, and then its position is sure.
    // A spot can have a third number: the power to ARRIVE there with, to slow down before a corner.
    // Near-wall spot to the GARDEN corner (Auto3's TO_GARDEN): back off, right, in front of the corner, in.
    static final double[][] TO_GARDEN = {{60, 29.5}, {30, 29.5}, {12, 18.5}, {4, -6.5}};
    static final double[][] TO_GARDEN_FROM_MIDDLE = {{30, 29.5}, {12, 18.5}, {4, -6.5}};  // after crossing back
    // The last line runs into the corner of our alliance wall and the near wall: pressed there, the robot
    // knows its X AND its Y again (ROBOT_SIDE from the alliance wall, ROBOT_HALF from the near wall).
    // Home along the near wall (Auto3's HOME): 3 in. off the wall, along it, into it at the near-wall spot.
    static final double[][] HOME = {{10, 11.5}, {60, 11.5, 0.3}, {60, 2.5}};  // 0.3: slow into the corner, don't overshoot toward the midline
    // Near-wall spot to the far side, under the HIVE (Auto3's GREEN path). It stops 14.5 in. from the far
    // wall, turns around, then drives into the far wall (FAR_PUSH). It turns at TURN_X, not 62: while
    // turning, a corner of the robot sticks out up to 12 in. from its middle (8.5 x 1.41), and at 62
    // that corner would cross the midline (G402).
    static final double[][] NEAR_TO_FAR = {{60, 100.5}, {TURN_X, 129.5}};
    // Into the far wall straight behind our far CELL (X 59), not at the start spot's 62: no big aim turn
    // needed, and odometry errors can't push it toward the midline.
    // Over to X 59 first, THEN straight into the wall: sliding sideways while pushing a wall makes the
    // wheels slip, and odometry loses the X (the simulator showed up to 6 in.).
    static final double[][] FAR_PUSH = {{FAR_CELL_X, 129.5}, {FAR_CELL_X, 144 - ROBOT_HALF + 3}};
    // Far-wall spot back to the near side, under the HIVE, then turn and into the near wall.
    static final double[][] FAR_TO_NEAR = {{62, 104}, {60, 40}, {TURN_X, 14.5}};
    static final double[][] NEAR_PUSH = {{60, 14.5}, {60, ROBOT_HALF - 3}};  // over first, then straight in
    // Far-wall spot back to the near side, stopping just past the HIVE to turn and go to the GARDEN.
    static final double[][] FAR_TO_MIDDLE = {{62, 104}, {TURN_X, 36}};
    // Side-wall spot: off the wall, turn, then to the near-wall or far-wall spot round the HIVE frame.
    static final double[][] SIDE_OFF = {{20, 72}};
    static final double[][] SIDE_TO_NEAR = {{30, 40}, {60, 14.5}};
    static final double[][] SIDE_TO_FAR = {{30, 104}, {62, 129.5}};
    // Parking: the LOADING ZONE is a GUESS (test T10). The robot only has to be partly in it and touch no wall.
    static final double[] PARK_SPOT = {14, 131};
    static final double[][] PARK_FROM_FAR = {{FAR_CELL_X, 131}, PARK_SPOT};                       // off the wall, along it
    static final double[][] PARK_FROM_NEAR = {{60, 14}, {28, 26}, {28, 124}, PARK_SPOT};  // round the HIVE frame
    static final double LEAVE_GAP = 4.5;  // LEAVE: back this far off the wall, so it touches nothing

    // THE CLOCK: estimated seconds for each block (from the plan, NOT measured: test T6).
    static final double AUTO_SECONDS = 30;
    static final double T_FIRST_SHOT = 5.6;    // flywheel up, look, aim, shoot 4, tip check
    static final double T_SHOT = 4.6;          // look, aim, shoot 4, tip check
    static final double T_GARDEN = 6.5;        // near-wall spot -> GARDEN -> home
    static final double T_CROSS = 6.0;         // under the HIVE, turn, into the other wall
    static final double T_CROSS_GARDEN = 10.0; // far-wall spot -> under the HIVE -> GARDEN -> home
    static final double T_SIDE = 4.0;          // side-wall spot -> near-wall or far-wall spot
    static final double T_PARK_FAR = 3.0, T_PARK_NEAR = 5.0, T_LEAVE = 0.5;
    static final double CLOCK_MARGIN = 0.5;    // seconds kept spare on top of every estimate

    // Shooting (as Auto3). The siphon can't count its turns: if it does too many or too few, change SECONDS_PER_TURN.
    static final double RPM = 2800, SIPHON_POWER = 0.3, SECONDS_PER_TURN = 1.5;
    static final double TIP_WAIT = 0.6;  // seconds after the last ball before the tip check looks (balls still falling)

    // ================================================================ THE CAMERA
    // CAMERA_BACK, CAMERA_HEIGHT, CAMERA_PITCH and the AIM_ numbers mean the same as in Auto4. The webcam
    // sits where the Limelight was, so they start the same; if it sits elsewhere, measure them again.
    static final boolean CAMERA_DECIDES = true;  // false = the camera only reports, the robot follows the plan blind
    static final boolean CAMERA_TURNS = true;    // false = it never turns onto the tags before a shot
    static final String CAMERA_NAME = "Webcam 1";  // its name in the Robot Configuration (the name Scan gives it)
    static final int CAMERA_WIDTH = 640, CAMERA_HEIGHT_PX = 480;  // the SDK has the C270's lens numbers only for 640x480
    // How small a tag it still finds: the picture is shrunk by this before the tags are searched.
    // 1 = not shrunk: the smallest / furthest tags, the fewest pictures a second. 2 = about twice the
    // pictures. 3 = the SDK's normal setting, the fastest, the shortest range.
    static final float TAG_DECIMATION = 2;
    // A fixed exposure keeps the picture the same in a dark gym and a bright one, and a short one doesn't
    // blur. 0 = the webcam's own automatic exposure. GAIN: 0 to 255 on the C270. Both are guesses.
    static final int EXPOSURE_MS = 8, GAIN = 200;
    static final double TAG_SIZE = 3.25;       // inches, the black square of each HIVE tag
    static final double CAMERA_BACK = 8.0;     // inches behind the robot's middle (guess: test T1)
    static final double CAMERA_HEIGHT = 8.0;   // inches above the floor (guess: test T1), as in the old Auto4
    static final double CAMERA_PITCH = 15.0;   // degrees tilted up (guess: test T1)
    static final double AIM_TRIM = 0.0, AIM_CLOSE = 2.5, AIM_MAX = 10, AIM_TURN_TIME = 1.5;
    static final double FAR_CELL = 65;         // inches: the CELL straight behind is closer than this (ours ~53)
    // seconds: stand still, then look this long. Longer than Auto4's 0.3: the webcam gives fewer
    // pictures a second than the Limelight, and a look wants several of them.
    static final double LOOK_SETTLE = 0.15, LOOK_TIME = 0.4;
    // Which CELL is up: the up CELL's tags are higher. Both CELLs seen: the higher one is up, if it is
    // higher by more than UP_MARGIN. GUESSES until test T3 (tip the HIVE by hand, read the "Up" line).
    static final double UP_MARGIN = 2.0;       // inches
    static final int UP_MIN_TAGS = 2;          // tags of a CELL needed before its height counts
    // Only ONE CELL seen: its tags higher than ONE_CELL_SPLIT = it is up, lower = it is down.
    // 0 = off (needs both CELLs). Set it from test T3: halfway between the up and down heights.
    static final double ONE_CELL_SPLIT = 0;
    // IS THE CELL BEHIND UP? (Changed after run 1, 2026-11-22: the HIVE did not tip, but only that one
    // CELL was in view, so the check above couldn't tell, assumed a tip, and the robot drove off to shoot
    // a CELL that was down.) Now it goes by what the camera really sees, compared with a reference:
    //   - From a shooting spot, an UP CELL straight behind shows its tags at a certain height. That height
    //     is learned on the first look at a CELL believed up (normally the near CELL at START).
    //   - Right after a volley, the look just before it, from the same spot, is the reference.
    // Its tags at the reference height (within TIP_MOVE) = UP. Much higher or lower, or gone while the
    // camera does give pictures = DOWN. Too few pictures = can't tell, and the plan decides.
    static final double TIP_MOVE = 2.0;        // inches: tags this much higher or lower = the CELL moved
    static final int MIN_PICTURES = 3;         // fewer pictures than this in a look = the camera can't tell
    // ODOMETRY + CAMERA. The camera does not decide alone: a camera fix only moves the odometry part of
    // the way, a big jump is thrown out, and the aim checks the camera against the odometry.
    static final double FIX_WEIGHT = 0.6;      // how much of the camera's X fix is believed (0 = never, 1 = all)
    static final double FIX_MAX = 12;          // inches: a camera fix further than this from odometry is a bad look
    static final double HEADING_WEIGHT = 0.5;  // how much of the camera's heading fix is believed
    static final double HEADING_MIN = 3;       // degrees: smaller = camera noise, the IMU is kept
    static final double HEADING_MAX = 8;       // degrees: a heading fix bigger than this is a bad look
    static final double REALIGN_MIN = 2.5;     // inches: further than this from straight in front of the CELL = slide over first
    static final double AIM_AGREE = 6;         // degrees: camera and odometry aims closer than this = use the camera
    // The HIVE tags are ids 30-45: 4 in a row under each CELL, the CELLS in this order (from the SDK's tag list).
    static final String[] CELLS = {"RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING"};
    static final double[] TAG_PLACE = {-6.5, -2.75, 2.75, 6.5};

    // ================================================================ DRIVING (as Auto3)
    static final double DRIVE_POWER = 1.0;
    static final double LINE_GAIN = 0.1, MAX_LINE_FIX = 0.5;
    static final double TWIST_GAIN = 0.02, MAX_TWIST_FIX = 0.3;
    static final double SLOW_ZONE = 24, MIN_POWER = 0.1;
    static final double SETTLE_TIME = 0.75;
    static final double WALL_SETTLE = 0.2;  // seconds on a wall before odometry is told where the wall is
    static final double TURN_CLOSE = 5, TURN_SETTLE = 1.0;
    static final double INCH_PER_TICK = Math.PI * 96 / 25.4 / 537.7;  // 96 mm wheels, 537.7 ticks/turn
    static final double TURN_RADIUS = (350 + 300) / 2.0 / 25.4;
    // Mecanum wheels slip sideways: the encoders count more than the robot really strafes. Test: strafe
    // 48 in. by the screen, measure with a tape, STRAFE_FACTOR = measured / 48. (Guess until measured.)
    static final double STRAFE_FACTOR = 0.9;
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    static final double HUB_EDGE = 70, IMU_STUCK = 100;

    // ================================================================ THE PICKS AND THE PLAN'S STATE

    static final int NEAR = 0, FAR = 1, SIDE = 2;                    // start spots / where the robot is
    static final int UP_NEAR = 0, UP_FAR = 1, UP_UNKNOWN = 2;        // which CELL is up
    static final int PICK_CAMERA = 0, PICK_NEAR = 1, PICK_FAR = 2;   // the X pick
    static final String[] SPOT_NAMES = {"NEAR-WALL SPOT", "FAR-WALL SPOT", "SIDE-WALL SPOT"};
    static final String[] UP_NAMES = {"NEAR CELL UP", "FAR CELL UP", "CAN'T TELL"};

    boolean blue = false;
    int startSpot = NEAR;
    int upPick = PICK_CAMERA;

    int spot;           // where the robot is now: NEAR, FAR or SIDE
    int held;           // POLLEN in the robot (it can't sense them: 4 at START, 0 after a volley, 4 after the GARDEN)
    boolean gardenUsed;
    int upBelief;       // which CELL it thinks is up now
    String upFrom = ""; // why it thinks so, for the screen
    boolean crossedSinceShot;
    boolean crossedBack;  // already crossed back once because the CELL on that side was down
    double upHeight = Double.NaN;      // how high an UP CELL's tags look from a shooting spot (NaN = not seen yet)
    double beforeHeight = Double.NaN;  // how high the shot CELL's tags looked just before the last volley
    int tips;           // volleys the tip check said (or assumed) tipped the HIVE
    String doing = "";
    final List<String> stepsDone = new ArrayList<>();
    final ElapsedTime match = new ElapsedTime();

    enum Act { TO_NEAR_SPOT, TO_FAR_SPOT, SHOOT, CROSS, GARDEN, FINISH }

    // ================================================================ HARDWARE AND ODOMETRY STATE

    DcMotor[] wheels;  // FL, FR, BL, BR
    DcMotor intake;
    CRServo leftFeeder, rightFeeder, siphon;
    DcMotorEx flywheel;
    IMU imu;
    String imuNote = "";

    // Odometry: where the robot is (X, Y) and where it faces, in "our side" numbers (see the top).
    // plan = where it should be after the last block; target / offLine are for the screen.
    String fixNote = "";
    int fixFromCam;
    double posX, posY, heading, startHeading, planX, planY, planHeading, targetX, targetY, offLine;
    final int[] startTicks = new int[4], lastTicks = new int[4];
    final double[] power = new double[4], evidence = new double[4], encoderSign = {1, 1, 1, 1};
    final double[] countedTicks = new double[4];
    final boolean[] dead = new boolean[4];
    double wheelHeading;
    double hubPitch, hubRoll, lastYaw, lastPitch, lastRoll, imuStuckInches;
    int wheelFixes = 0;
    boolean turning = false;
    boolean[][] been = new boolean[MAP_ROWS][MAP_COLS];

    // Camera state. portal == null: no webcam in the Robot Configuration, and it runs the route blind.
    VisionPortal portal;          // the webcam's picture
    AprilTagProcessor tagFinder;  // finds the AprilTags in it
    boolean exposureSet = false;  // EXPOSURE_MS / GAIN sent (once the webcam streams)
    String cameraNote = "", cameraSetup = "", upNote = "", initCheck = "";
    final List<String> lookNotes = new ArrayList<>();  // what each look before / after a volley said
    final double[][] tagSeen = new double[16][4];      // per tag 30-45: inches behind, right, up (added up), times seen
    int pictures;
    int cameraUp = UP_UNKNOWN;  // the camera's answer from the last look
    final ElapsedTime watch = new ElapsedTime();

    // RUN LOG, to find out afterwards what went wrong. From START: one line every loop with where
    // odometry says the robot is, where it is driving to, and what the wheels, intake and siphon are
    // told; plus a line for every step, drive / turn command, camera look and note. The file is
    // /sdcard/FIRST/data/auto5-log-N.csv on the Control Hub, N counting up with every run; the "Log"
    // line on the screen names it. Plug the Control Hub into the laptop to fetch it.
    static final boolean LOG = true;
    BufferedWriter log;
    String logName = "starts at START";
    double cmdForward, cmdRight, cmdTurn, intakeCmd, siphonCmd, rpmNow;
    long lastLoopNanos;
    final ElapsedTime logFlush = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);  // so the field drawing lines up
        setUp();
        startOdometry(spotPose(startSpot));
        while (opModeInInit()) {
            readPicks();
            updateOdometry();
            watchHive();
            showInit();
        }

        // ------------------------------------------------------------ START
        match.reset();
        openLog();
        logRow("init", String.format("%s; %s %s; up pick %s; camera says %s; camera check: %s; up: %s; webcam: %s %s",
                blue ? "BLUE" : "RED", SPOT_NAMES[startSpot], tile(blue, startSpot),
                upPick == PICK_NEAR ? "NEAR" : upPick == PICK_FAR ? "FAR" : "CAMERA", UP_NAMES[cameraUp],
                portal == null ? cameraNote : initCheck, upNote, portal == null ? "none" : webcamLine(), cameraSetup));
        try {
            runPlan();
        } finally {
            closeLog();
            if (portal != null) {
                portal.close();
            }
        }
    }

    /** AUTO itself, from START. */
    void runPlan() {
        startOdometry(spotPose(startSpot));
        flywheel.setVelocity(RPM * 28 / 60.0);  // stays on the whole time
        findTags(false);  // the tag finder only runs during a look: odometry gets the Control Hub while driving
        spot = startSpot;
        held = 4;
        gardenUsed = false;
        crossedSinceShot = false;
        tips = 0;
        upBelief = startUp();
        upFrom = upPick != PICK_CAMERA ? "driver's X pick"
                : cameraUp != UP_UNKNOWN && CAMERA_DECIDES ? "camera in INIT" : "normal setup (camera can't tell)";

        // ------------------------------------------------------------ THE PLAN: decide, do, repeat
        while (opModeIsActive()) {
            Act next = decide(spot, held, gardenUsed, upBelief, timeLeft(), match.seconds() < 1, crossedSinceShot, !crossedBack);
            if (next == Act.FINISH) {
                break;
            }
            switch (next) {
                case TO_NEAR_SPOT:
                case TO_FAR_SPOT:
                    fromSide(next == Act.TO_FAR_SPOT ? FAR : NEAR);
                    break;
                case SHOOT:
                    shootHere();
                    break;
                case CROSS:
                    if (crossedSinceShot) {
                        crossedBack = true;  // the CELL on this side was down: back to the other one, once
                        note("the CELL here is down: crossing back to shoot the other one");
                    }
                    cross();
                    crossedSinceShot = true;
                    break;
                case GARDEN:
                    collectGarden();
                    break;
                default:
                    break;
            }
        }
        finish();
        while (opModeIsActive()) {  // keeps the debug screen up until AUTO ends
            updateOdometry();
            show("done");
        }
    }

    // ================================================================ THE BRAIN

    /**
     * What to do next. Used twice: before START to draw and list the plan (pretending every volley
     * tips the HIVE), and during AUTO with what the camera really saw.
     *   On the side-wall spot: go to the spot right behind the CELL that is up.
     *   Holding POLLEN: the CELL right behind us is up -> shoot. It is down -> cross to the other spot
     *       (straight back again only once, mayCrossBack: one of our two CELLs is always up, so
     *       after that it shoots wherever it is).
     *   Empty: collect the GARDEN once (it ends on the near-wall spot, the start spot). Used already: finish.
     *       So when a volley did NOT tip the HIVE, the CELL behind is still up: GARDEN, back to the
     *       start spot, and shoot the same CELL again.
     * Every step must fit in the time left (the clock rule), or it finishes now.
     */
    static Act decide(int spot, int held, boolean gardenUsed, int up, double timeLeft, boolean firstShot,
                      boolean crossedSinceShot, boolean mayCrossBack) {
        if (up == UP_UNKNOWN) {
            up = UP_NEAR;  // the normal setup
        }
        if (spot == SIDE) {
            return fits(timeLeft, T_SIDE + T_SHOT) ? (up == UP_FAR ? Act.TO_FAR_SPOT : Act.TO_NEAR_SPOT) : Act.FINISH;
        }
        int behind = spot == NEAR ? UP_NEAR : UP_FAR;  // the CELL straight behind the robot
        if (held > 0) {
            if (up == behind) {
                return fits(timeLeft, firstShot ? T_FIRST_SHOT : T_SHOT) ? Act.SHOOT : Act.FINISH;
            }
            if (crossedSinceShot && !mayCrossBack) {
                return Act.FINISH;
            }
            return fits(timeLeft, T_CROSS + T_SHOT) ? Act.CROSS : Act.FINISH;
        }
        if (gardenUsed) {
            return Act.FINISH;
        }
        return fits(timeLeft, (spot == NEAR ? T_GARDEN : T_CROSS_GARDEN) + T_SHOT) ? Act.GARDEN : Act.FINISH;
    }

    /** The clock rule: this many seconds, plus backing off the wall, plus a margin, still fit. */
    static boolean fits(double timeLeft, double seconds) {
        return timeLeft >= seconds + T_LEAVE + CLOCK_MARGIN;
    }

    /** At the end: park if there is time, else LEAVE. */
    static boolean parkFits(int spot, double timeLeft) {
        return timeLeft >= (spot == FAR ? T_PARK_FAR : T_PARK_NEAR) + CLOCK_MARGIN;
    }

    double timeLeft() {
        return AUTO_SECONDS - match.seconds();
    }

    /** Which CELL it assumes is up at START: the driver's X pick, else the camera's, else the normal setup. */
    int startUp() {
        if (upPick == PICK_NEAR) return UP_NEAR;
        if (upPick == PICK_FAR) return UP_FAR;
        if (CAMERA_DECIDES && cameraUp != UP_UNKNOWN) return cameraUp;
        return UP_NEAR;
    }

    /** One planned step, for the INIT screen and the map. */
    static class PlanStep {
        String word;
        double seconds;
        double[][][] paths;  // the lines it drives, in "our side" inches
        PlanStep(String word, double seconds, double[][]... paths) {
            this.word = word;
            this.seconds = seconds;
            this.paths = paths;
        }
    }

    /** The whole plan for these picks, pretending every volley tips the HIVE and every step takes its T_ time. */
    static List<PlanStep> planFor(int start, int up) {
        List<PlanStep> plan = new ArrayList<>();
        int spot = start, held = 4;
        boolean garden = false, crossed = false;
        double t = 0;
        if (up == UP_UNKNOWN) up = UP_NEAR;
        for (int i = 0; i < 12; i++) {
            Act a = decide(spot, held, garden, up, AUTO_SECONDS - t, t < 1, crossed, false);
            PlanStep s;
            if (a == Act.FINISH) break;
            if (a == Act.TO_NEAR_SPOT || a == Act.TO_FAR_SPOT) {
                boolean far = a == Act.TO_FAR_SPOT;
                s = new PlanStep(far ? "to far spot" : "to near spot", T_SIDE, prepend(SIDE_SPOT, SIDE_OFF),
                        prepend(SIDE_OFF[0], far ? SIDE_TO_FAR : SIDE_TO_NEAR));
                spot = far ? FAR : NEAR;
            } else if (a == Act.SHOOT) {
                s = new PlanStep(spot == NEAR ? "shoot near" : "shoot far", t < 1 ? T_FIRST_SHOT : T_SHOT);
                held = 0;
                crossed = false;
                up = up == UP_NEAR ? UP_FAR : UP_NEAR;  // pretend it tipped
            } else if (a == Act.CROSS) {
                s = spot == NEAR ? new PlanStep("cross", T_CROSS, prepend(NEAR_SPOT, NEAR_TO_FAR))
                        : new PlanStep("cross", T_CROSS, prepend(FAR_SPOT, FAR_TO_NEAR));
                spot = spot == NEAR ? FAR : NEAR;
                crossed = true;
            } else {  // GARDEN
                s = spot == NEAR ? new PlanStep("GARDEN", T_GARDEN, prepend(NEAR_SPOT, TO_GARDEN), HOME)
                        : new PlanStep("cross + GARDEN", T_CROSS_GARDEN, prepend(FAR_SPOT, FAR_TO_MIDDLE),
                                prepend(FAR_TO_MIDDLE[1], TO_GARDEN_FROM_MIDDLE), HOME);
                spot = NEAR;
                held = 4;
                garden = true;
            }
            plan.add(s);
            t += s.seconds;
        }
        if (parkFits(spot, AUTO_SECONDS - t)) {
            plan.add(spot == FAR ? new PlanStep("park", T_PARK_FAR, prepend(FAR_SPOT, PARK_FROM_FAR))
                    : new PlanStep("park", T_PARK_NEAR, prepend(NEAR_SPOT, PARK_FROM_NEAR)));
        } else {
            plan.add(new PlanStep("LEAVE", T_LEAVE));
        }
        return plan;
    }

    static double[][] prepend(double[] first, double[][] rest) {
        double[][] all = new double[rest.length + 1][];
        all[0] = first;
        System.arraycopy(rest, 0, all, 1, rest.length);
        return all;
    }

    // ================================================================ BLOCKS

    /** Side-wall spot to the near-wall or far-wall spot: off the wall, turn, round the HIVE frame, into the wall. */
    void fromSide(int to) {
        step(to == FAR ? "to the far-wall spot" : "to the near-wall spot");
        drivePath(SIDE_OFF, false);
        turnTo(to == FAR ? 90 : 270);
        drivePath(to == FAR ? SIDE_TO_FAR : SIDE_TO_NEAR, false);
        pressWall(to);
        spot = to;
    }

    /** Under the HIVE to the other spot (Auto3's GREEN path, either way), turn around, into the wall. */
    void cross() {
        step("cross under the HIVE");
        if (spot == NEAR) {
            drivePath(NEAR_TO_FAR, false);
            turnTo(90);
            pressWall(FAR);
            spot = FAR;
        } else {
            drivePath(FAR_TO_NEAR, false);
            turnTo(270);
            pressWall(NEAR);
            spot = NEAR;
        }
    }

    /** Front into the near or far wall; pressed on it, the robot's Y is sure again. */
    void pressWall(int which) {
        drivePath(which == FAR ? FAR_PUSH : NEAR_PUSH, true);
        settleOnWall();
        posY = planY = which == FAR ? 144 - ROBOT_HALF : ROBOT_HALF;
    }

    /**
     * After a push into a wall the wheels still spin for a moment and slip, so the encoders count
     * inches the robot never moved. Wait till they stop, THEN tell odometry where the wall is.
     */
    void settleOnWall() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < WALL_SETTLE) {
            updateOdometry();
            show("pressed on the wall: wheels stopping");
        }
    }

    /** ONE motion into the GARDEN corner, the intake on for the last line, then home to the near-wall spot. */
    void collectGarden() {
        step(spot == NEAR ? "collect the GARDEN" : "cross back, collect the GARDEN");
        if (spot == FAR) {
            drivePath(FAR_TO_MIDDLE, false);  // under the HIVE, stopping just past it
            turnTo(270);                      // intake toward the GARDEN
            drivePath(TO_GARDEN_FROM_MIDDLE, true, TO_GARDEN_FROM_MIDDLE.length - 1);
        } else {
            drivePath(TO_GARDEN, true, TO_GARDEN.length - 1);
        }
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 0.25) {  // pressed on the wall: let the balls in
            updateOdometry();
            show("collecting");
        }
        setIntake(0);
        posY = planY = ROBOT_HALF;  // front pressed on the near wall
        posX = planX = ROBOT_SIDE;  // right side pressed on our alliance wall
        note("GARDEN corner: odometry reset to the corner");
        drivePath(HOME, true);
        settleOnWall();
        posY = planY = ROBOT_HALF;
        spot = NEAR;
        held = 4;
        gardenUsed = true;
    }

    /**
     * At the near-wall or far-wall spot: look first. The CELL straight behind is up (or the camera
     * can't tell): aim, shoot 4, tip check. It is down: don't shoot (the brain then crosses).
     * Just crossed back because the other CELL was down: shoot here whatever the camera says, since
     * one of our two CELLs is always up.
     */
    void shootHere() {
        int behind = spot == NEAR ? UP_NEAR : UP_FAR;
        String cell = behind == UP_NEAR ? "near" : "far";
        step("look, shoot the " + cell + " CELL");
        double pathHeading = planHeading;
        ElapsedTime timer = new ElapsedTime();
        if (portal != null) {
            lookAtHive("is the " + cell + " CELL up?");
            int seen = behindState(behind, upHeight);
            note("before shot " + (tips + 1) + ": " + upNote);
            if (seen != UP_UNKNOWN && seen != behind && CAMERA_DECIDES) {
                if (crossedBack && crossedSinceShot) {
                    note("  -> it says down here too, but one of our CELLs must be up: shooting anyway");
                    seen = UP_UNKNOWN;
                } else {
                    // The camera says: don't shoot here. That changes the plan, so look once more first.
                    lookAtHive("look again: is the " + cell + " CELL really down?");
                    int again = behindState(behind, upHeight);
                    note("  second look: " + upNote);
                    if (again != seen) {
                        note("  -> the two looks don't agree: follow the plan, shoot");
                        seen = UP_UNKNOWN;
                    }
                }
            }
            if (seen != UP_UNKNOWN && CAMERA_DECIDES) {
                upBelief = seen;
                upFrom = "camera before the shot";
                if (seen != behind) {
                    note("  -> the " + cell + " CELL is DOWN: not shooting here");
                    return;
                }
            }
        }
        // Realign: odometry (just fixed by the camera) says the robot is not straight in front of the
        // CELL. Slide along the wall till it is: a straight shot, and well away from the midline.
        double cellX = (behind == UP_NEAR ? NEAR_CELL_XY : FAR_CELL_XY)[0];
        double sideOff = posX - cellX;
        if (Math.abs(sideOff) > REALIGN_MIN && Math.abs(sideOff) < FIX_MAX) {
            note(String.format("  realign: %.1f in. %s of the CELL, sliding along the wall", Math.abs(sideOff),
                    sideOff > 0 ? "toward the midline" : "toward our wall"));
            drivePath(new double[][] {{cellX, planY}}, false);
            if (portal != null) {
                lookAtHive("look again after sliding");
            }
        }
        if (portal != null) {
            aimAt(behind);
            // What the CELL looks like right now, just before the volley: the tip check compares with it.
            // The first time a CELL we shoot at is seen, its height is also what "UP" looks like.
            CellLook now = cellLook(behind == UP_NEAR ? nearCellIndex() : farCellIndex());
            beforeHeight = now.tags > 0 ? now.up : Double.NaN;
            if (Double.isNaN(upHeight) && now.tags > 0) {
                upHeight = now.up;
                note(String.format(Locale.US, "  an UP CELL's tags look %.1f in. high from a shooting spot: that is the reference now", upHeight));
            }
        }
        while (opModeIsActive() && timer.seconds() < 2 && rpmNow < RPM - 75) {
            updateOdometry();
            show("flywheel speeding up");
        }
        setIntake(1);
        spinSiphon(1);    // one turn shoots all 4 balls
        spinSiphon(-0.25);
        spinSiphon(0.5);  // safety: makes sure every ball is out
        setIntake(0);
        planHeading = pathHeading;
        held = 0;
        crossedSinceShot = false;
        tipCheck(behind);
    }

    /**
     * About a second after a volley: did the HIVE tip? The robot hasn't moved, so the look just before
     * the volley is the reference: the same CELL's tags still at the same height = NOT tipped (then the
     * brain collects the GARDEN, comes back to this spot and shoots the same CELL again). Its tags
     * moved or gone = TIPPED. No answer at all (no pictures) = assume it did (the plan).
     */
    void tipCheck(int shotAt) {
        int other = shotAt == UP_NEAR ? UP_FAR : UP_NEAR;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < TIP_WAIT) {  // balls still landing; don't drive under it
            updateOdometry();
            show("waiting for the HIVE to settle");
        }
        double reference = Double.isNaN(beforeHeight) ? upHeight : beforeHeight;
        int seen = UP_UNKNOWN;
        if (portal != null) {
            lookAtHive("did it tip?");
            seen = behindState(shotAt, reference);
            if (seen == shotAt && CAMERA_DECIDES) {
                // "NOT tipped" changes the plan (it shoots this CELL again): look once more first.
                note("tip check, first look: " + upNote);
                lookAtHive("look again: really not tipped?");
                if (behindState(shotAt, reference) != shotAt) {
                    note("  the two looks don't agree: follow the plan");
                    seen = UP_UNKNOWN;
                }
            }
        }
        if (seen == shotAt && CAMERA_DECIDES) {
            upBelief = shotAt;
            upFrom = "tip check: NOT tipped";
            note("tip check: NOT TIPPED (" + upNote + ")");
        } else {
            upBelief = other;
            tips++;
            String said = seen == other ? "TIPPED"
                    : seen == shotAt ? "camera says NOT tipped, ignored (CAMERA_DECIDES off): assumed tipped"
                    : "no answer, assumed tipped";
            upFrom = "tip check: " + said;
            note("tip check: " + said + " (" + upNote + ")");
        }
    }

    /** The end: park in the LOADING ZONE if it fits, else back a few inches off the wall (LEAVE). */
    void finish() {
        if (spot == SIDE) {
            step("LEAVE");
            drivePath(new double[][] {{posX + LEAVE_GAP, posY}}, false);
            return;
        }
        if (parkFits(spot, timeLeft())) {
            step("park in the LOADING ZONE");
            drivePath(spot == FAR ? PARK_FROM_FAR : PARK_FROM_NEAR, false);
        } else {
            step("LEAVE: off the wall");
            drivePath(new double[][] {{planX, planY + (spot == FAR ? -LEAVE_GAP : LEAVE_GAP)}}, false);
        }
    }

    void step(String what) {
        stepsDone.add(String.format("%4.1f s  %s", match.seconds(), what));
        doing = what;
        logRow("step", what);
    }

    void note(String what) {
        lookNotes.add(what);
        if (lookNotes.size() > 6) {
            lookNotes.remove(0);
        }
        logRow("note", what);
    }

    // ================================================================ RUN LOG (see LOG)

    void openLog() {
        if (!LOG) {
            logName = "off (LOG = false)";
            return;
        }
        try {
            File dir = AppUtil.ROBOT_DATA_DIR;
            dir.mkdirs();
            File file;
            int n = 1;
            do {
                file = new File(dir, "auto5-log-" + n + ".csv");
                n++;
            } while (file.exists());
            log = new BufferedWriter(new FileWriter(file));
            logName = file.getPath();
            log.write("t,kind,step,x,y,heading,wheelHeading,targetX,targetY,offLine,cmdForward,cmdRight,cmdTurn,"
                    + "pFL,pFR,pBL,pBR,ticksFL,ticksFR,ticksBL,ticksBR,rpm,intake,siphon,tagFinder,loopMs,text\n");
        } catch (IOException e) {
            log = null;
            logName = "could not open the log: " + e.getMessage();
        }
    }

    /**
     * One line of the log. kind "pos" = the every-loop line (odometry, target, commands); any other
     * kind (init, step, cmd, look, note) = an event, its words in the last column.
     */
    void logRow(String kind, String text) {
        if (log == null) {
            return;
        }
        boolean pos = kind.equals("pos");
        long now = System.nanoTime();
        double loopMs = 0;
        if (pos) {
            loopMs = lastLoopNanos == 0 ? 0 : (now - lastLoopNanos) / 1e6;
            lastLoopNanos = now;
        }
        StringBuilder r = new StringBuilder(220);
        add(r, "%.3f", match.seconds());
        add(r, "%s", kind);
        add(r, "%s", csv(doing));
        add(r, "%.2f", posX);
        add(r, "%.2f", posY);
        add(r, "%.1f", heading);
        if (pos) {
            add(r, "%.1f", wheelHeading);
            add(r, "%.1f", targetX);
            add(r, "%.1f", targetY);
            add(r, "%.2f", offLine);
            add(r, "%.3f", cmdForward);
            add(r, "%.3f", cmdRight);
            add(r, "%.3f", cmdTurn);
            for (int i = 0; i < 4; i++) {
                add(r, "%.3f", power[i]);
            }
            for (int i = 0; i < 4; i++) {
                add(r, "%d", lastTicks[i] - startTicks[i]);
            }
            add(r, "%.0f", rpmNow);
            add(r, "%.2f", intakeCmd);
            add(r, "%.2f", siphonCmd);
            add(r, "%s", portal != null && portal.getProcessorEnabled(tagFinder) ? "on" : "off");
            add(r, "%.1f", loopMs);
        } else {
            for (int i = 0; i < 20; i++) {
                r.append(',');
            }
        }
        add(r, "%s", csv(text));
        try {
            log.write(r.append('\n').toString());
            if (!pos || logFlush.seconds() > 0.5) {  // on the Control Hub's storage at least twice a second
                log.flush();
                logFlush.reset();
            }
        } catch (IOException e) {
            closeLog();
            logName = "log stopped: " + e.getMessage();
        }
    }

    static void add(StringBuilder r, String format, Object value) {
        if (r.length() > 0) {
            r.append(',');
        }
        r.append(String.format(Locale.US, format, value));
    }

    static String csv(String text) {
        return text.replace(',', ';').replace('\n', ' ');
    }

    void closeLog() {
        if (log == null) {
            return;
        }
        try {
            log.close();
        } catch (IOException e) {
            // nothing more to save
        }
        log = null;
    }

    // ================================================================ DRIVING (Auto3's, in "our side" numbers)

    void drivePath(double[][] spots, boolean endsAtWall) {
        drivePath(spots, endsAtWall, -1);
    }

    /**
     * ONE motion through these spots: straight lines, full speed, no stopping at the corners, never
     * turning. Every loop odometry says where the robot really is; the wheel powers are: full speed
     * along the line, a push back onto the line (LINE_GAIN per inch off it), a turn back to the
     * heading (TWIST_GAIN per degree). endsAtWall = the last line runs into a wall, which stops it.
     * Otherwise it eases onto the last spot. Blocked (another robot): the rest of the path is skipped.
     * The intake comes on when line number intakeFromSpot starts (-1 = never).
     */
    void drivePath(double[][] spots, boolean endsAtWall, int intakeFromSpot) {
        for (int i = 0; i < spots.length; i++) {
            if (i == intakeFromSpot) {
                setIntake(1);
            }
            boolean last = i == spots.length - 1;
            double arrive = spots[i].length > 2 ? spots[i][2] : 1;
            if (!driveLine(spots[i][0], spots[i][1], last && !endsAtWall, arrive)) {
                break;
            }
        }
        setWheels(0, 0, 0);
    }

    /** One line of a path; the wheels keep running at the end unless easeIn. False if blocked. */
    boolean driveLine(double toX, double toY, boolean easeIn, double arrivePower) {
        toX = Math.min(toX, MAX_X);  // G402: never past the midline
        targetX = toX;
        targetY = toY;
        double fromX = planX, fromY = planY;
        double length = Math.hypot(toX - fromX, toY - fromY);
        double ux = (toX - fromX) / Math.max(length, 0.001);  // direction of the line
        double uy = (toY - fromY) / Math.max(length, 0.001);
        logRow("cmd", String.format(Locale.US, "drive line from X %.1f Y %.1f to X %.1f Y %.1f (%.1f in.)%s%s",
                fromX, fromY, toX, toY, length, easeIn ? " easing in" : "", arrivePower < 1 ? " arrive at power " + arrivePower : ""));
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stuck = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        ElapsedTime close = new ElapsedTime();
        double furthest = -Double.MAX_VALUE;
        boolean blocked = false;
        while (opModeIsActive() && timer.seconds() < 2 + length / 8) {
            updateOdometry();
            double movedX = posX - fromX, movedY = posY - fromY;
            double along = movedX * ux + movedY * uy;  // how far along the line
            offLine = movedX * uy - movedY * ux;       // how far right of the line (seen driving along it)
            double toGo = length - along;
            if (!easeIn && toGo < 1) {
                break;  // at the corner: straight on to the next line
            }
            if (easeIn && Math.hypot(toGo, offLine) < 0.5) {
                if (settled.seconds() > 0.1) {
                    break;
                }
            } else {
                settled.reset();
            }
            if (easeIn && Math.abs(toGo) < 1) {
                if (close.seconds() > SETTLE_TIME) {
                    break;
                }
            } else {
                close.reset();
            }
            if (along > furthest + 0.25 || Math.abs(toGo) < 1) {
                furthest = Math.max(furthest, along);
                stuck.reset();
            } else if (stuck.seconds() > 0.6) {
                blocked = true;
                break;
            }
            double speed = DRIVE_POWER;
            if (easeIn) {
                speed = Range.clip(toGo / SLOW_ZONE, -1, 1) * DRIVE_POWER;
                if (Math.abs(toGo) > 0.5 && Math.abs(speed) < MIN_POWER) {
                    speed = Math.copySign(MIN_POWER, toGo);
                }
            } else if (arrivePower < 1) {
                speed = Math.max(arrivePower, Math.min(1, toGo / SLOW_ZONE)) * DRIVE_POWER;
            }
            // Field directions: along the line, plus a push back toward it (to its left when right of it).
            double fix = Range.clip(LINE_GAIN * offLine, -MAX_LINE_FIX, MAX_LINE_FIX);
            double vx = ux * speed - uy * fix;
            double vy = uy * speed + ux * fix;
            double twist = wrapDegrees(heading - planHeading);  // positive = turned too far left
            // Turned into the robot's own forward / right by its real heading.
            double h = Math.toRadians(heading);
            setWheels(vx * Math.cos(h) + vy * Math.sin(h),
                      vx * Math.sin(h) - vy * Math.cos(h),
                      Range.clip(TWIST_GAIN * twist, -MAX_TWIST_FIX, MAX_TWIST_FIX));
            show(String.format("driving to X %.0f, Y %.0f", toX, toY));
        }
        planX = blocked ? posX : toX;
        planY = blocked ? posY : toY;
        logRow("cmd", String.format(Locale.US, "line %s after %.2f s: odometry X %.1f Y %.1f, off the line %.1f in.",
                blocked ? "BLOCKED (stopped moving)" : timer.seconds() >= 2 + length / 8 ? "TIMED OUT" : "done",
                timer.seconds(), posX, posY, offLine));
        return !blocked;
    }

    void turnTo(double newHeading) {
        turnTo(newHeading, 5, false);
    }

    /** Turns on the spot (Auto3's). offTheWall = front on a wall: the side that goes back backs off it. */
    void turnTo(double newHeading, double timeLimit, boolean offTheWall) {
        logRow("cmd", String.format(Locale.US, "turn from %.1f to %.1f deg%s", heading, newHeading, offTheWall ? " (backing off the wall)" : ""));
        planHeading = newHeading;
        turning = true;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        ElapsedTime close = null;
        while (opModeIsActive() && timer.seconds() < timeLimit) {
            updateOdometry();
            double error = wrapDegrees(heading - planHeading);
            if (close == null && Math.abs(error) <= TURN_CLOSE) {
                close = new ElapsedTime();
            }
            if (close != null && close.seconds() > TURN_SETTLE) {
                break;
            }
            double turn = 0;
            if (Math.abs(error) <= 1.5) {
                if (settled.seconds() > 0.2) {
                    break;
                }
            } else {
                settled.reset();
                turn = turnPower(error);
            }
            setWheels(offTheWall ? -Math.abs(turn) : 0, 0, turn);
            show(String.format("turning to %.0f deg", wrapDegrees(planHeading)));
        }
        setWheels(0, 0, 0);
        turning = false;
        logRow("cmd", String.format(Locale.US, "turn ended after %.2f s at %.1f deg (%.1f off)", timer.seconds(), heading,
                wrapDegrees(heading - planHeading)));
        if (imu != null) {
            wheelHeading = heading;
        }
    }

    double turnPower(double degreesTooFarLeft) {
        double turn = Range.clip(0.01 * degreesTooFarLeft, -1.0, 1.0);
        return Math.abs(turn) < 0.2 ? Math.copySign(0.2, turn) : turn;
    }

    void spinSiphon(double turns) {
        siphonCmd = Math.signum(turns) * SIPHON_POWER;
        logRow("cmd", String.format(Locale.US, "siphon %+.2f turns at %.0f RPM", turns, rpmNow));
        siphon.setPower(siphonCmd);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < Math.abs(turns) * SECONDS_PER_TURN) {
            updateOdometry();
            show(String.format("shooting: siphon %s", turns > 0 ? "forward" : "backward"));
        }
        siphonCmd = 0;
        siphon.setPower(0);
    }

    void setIntake(double p) {
        intakeCmd = p;
        intake.setPower(p);
        leftFeeder.setPower(p);
        rightFeeder.setPower(p);
    }

    // ================================================================ CAMERA

    /** Tag row of our near / far CELL (0-3, see CELLS). Red: near = RED AUDIENCE. Blue: near = BLUE SCORING. */
    int nearCellIndex() {
        return blue ? 3 : 1;
    }

    int farCellIndex() {
        return blue ? 2 : 0;
    }

    /** What the camera found for one CELL in the last look. */
    static class CellLook {
        int tags;                    // of its 4 tags, how many were seen
        double behind, right, up;    // its middle: inches behind and right of the robot's middle, inches above the floor
    }

    /** Stands still and looks at the HIVE tags for LOOK_TIME. */
    void lookAtHive(String what) {
        findTags(true);
        ElapsedTime timer = new ElapsedTime();
        boolean looking = false;
        while (opModeIsActive() && timer.seconds() < LOOK_SETTLE + LOOK_TIME) {
            updateOdometry();
            if (!looking && timer.seconds() >= LOOK_SETTLE) {
                startLook();  // pictures from before were taken while the robot was still moving
                looking = true;
            }
            if (looking) {
                readTags();
            }
            show(what);
        }
        findTags(false);
        CellLook n = cellLook(nearCellIndex()), f = cellLook(farCellIndex());
        logRow("look", String.format(Locale.US, "%s: %d pictures (%s); near CELL %d tags %.1f behind %.1f right %.1f up;"
                        + " far CELL %d tags %.1f behind %.1f right %.1f up", what, pictures, portal.getCameraState(),
                n.tags, n.behind, n.right, n.up, f.tags, f.behind, f.right, f.up));
        fixFromCamera();
        logRow("look", "after the look: " + fixNote);
    }

    /** Switches the tag finder on (to look) or off (while driving: it takes a lot of the Control Hub's time). */
    void findTags(boolean on) {
        if (portal != null) {
            portal.setProcessorEnabled(tagFinder, on);
        }
    }

    /**
     * After a look: where the robot is, from where our two CELLs appear. Each CELL seen with 2 or more
     * tags gives a spot (the CELL's known spot, minus how far behind and right of us it looked). Only
     * the X is fixed: every look happens pressed on a wall, which already gives Y. The fix moves
     * odometry FIX_WEIGHT of the way and is ignored if it is more than FIX_MAX away (a bad look).
     */
    void fixFromCamera() {
        fixHeadingFromCamera();
        double h = Math.toRadians(heading);
        double sumX = 0;
        int n = 0;
        for (int which = 0; which < 2; which++) {
            CellLook c = cellLook(which == 0 ? nearCellIndex() : farCellIndex());
            if (c.tags < 2 || c.behind > 90) {
                continue;
            }
            double[] cell = which == 0 ? NEAR_CELL_XY : FAR_CELL_XY;
            // behind = toward heading + 180, right = toward heading - 90
            sumX += cell[0] + c.behind * Math.cos(h) - c.right * Math.sin(h);
            n++;
        }
        if (n == 0) {
            fixNote = "camera fix: no CELL seen well, odometry kept";
            return;
        }
        double camX = sumX / n;
        double jump = camX - posX;
        if (Math.abs(jump) > FIX_MAX) {
            fixNote = String.format("camera fix: X %.0f is %.0f in. from odometry: ignored", camX, jump);
            return;
        }
        posX += FIX_WEIGHT * jump;
        planX = posX;
        fixFromCam++;
        fixNote = String.format("camera fix %d: X moved %+.1f in. (camera said %.1f)", fixFromCam, FIX_WEIGHT * jump, camX);
    }

    /**
     * Both our CELLs seen well (3+ tags each): the line from our near CELL to our far CELL runs
     * straight along the field (+Y), so the way that line looks from the robot says which way the
     * robot really points. Fixes a heading that is off because the robot was put down crooked or got
     * knocked: the IMU can't know about either. Moves HEADING_WEIGHT of the way, ignores more than
     * HEADING_MAX. (One CELL alone was tried in the simulator: its 4 tags are too close together,
     * the answer jumped about 3 degrees from look to look, worse than no fix.)
     */
    void fixHeadingFromCamera() {
        CellLook n = cellLook(nearCellIndex()), f = cellLook(farCellIndex());
        if (n.tags < 3 || f.tags < 3) {
            return;
        }
        double fieldAngle = Math.toDegrees(Math.atan2(FAR_CELL_XY[1] - NEAR_CELL_XY[1], FAR_CELL_XY[0] - NEAR_CELL_XY[0]));
        double camHeading = fieldAngle - 180 - Math.toDegrees(Math.atan2(f.right - n.right, f.behind - n.behind));
        double off = wrapDegrees(camHeading - heading);
        if (Math.abs(off) > HEADING_MAX) {
            note(String.format("camera heading is %.1f deg from odometry: ignored", off));
            return;
        }
        if (Math.abs(off) < HEADING_MIN) {
            return;  // the IMU is better than the camera at small angles: only fix a real mistake
        }
        double turn = HEADING_WEIGHT * off;
        startHeading += turn;  // heading = startHeading + IMU yaw
        wheelHeading += turn;
        heading += turn;
        note(String.format("camera heading fix: %+.1f deg", turn));
    }

    /** INIT: twice a second, what the camera sees, which CELL is up, and does it match the picks. */
    void watchHive() {
        if (portal == null) {
            return;
        }
        readTags();
        if (watch.seconds() > 0.5) {
            cameraUp = whichIsUp();
            initCheck = checkPicks();
            startLook();
            watch.reset();
        }
    }

    void startLook() {
        for (double[] tag : tagSeen) {
            tag[0] = tag[1] = tag[2] = tag[3] = 0;
        }
        pictures = 0;
        tagFinder.getFreshDetections();  // throws away the picture already waiting: only pictures from now on count
    }

    /** Counts the tag finder's newest picture once and adds its HIVE tags to tagSeen (Auto4's math). */
    void readTags() {
        setExposure();
        List<AprilTagDetection> found = tagFinder.getFreshDetections();
        if (found == null) {
            return;  // no new picture since last time
        }
        pictures++;
        logRow("pic", String.format(Locale.US, "picture %d of this look: %d tags", pictures, found.size()));
        double pitch = Math.toRadians(CAMERA_PITCH);
        for (AprilTagDetection tag : found) {
            if (!(tag instanceof AprilTagSingleDetection)) {
                continue;
            }
            AprilTagSingleDetection one = (AprilTagSingleDetection) tag;
            int id = one.id;
            if (id < 30 || id > 45 || tag.rawPose == null) {
                logRow("tag", String.format(Locale.US, "id %d at pixel %.0f %.0f: not a HIVE tag, not used", id, one.center.x, one.center.y));
                continue;
            }
            // Camera space: x right, y down, z straight out of the lens, in inches. The same as the
            // Limelight's in Auto4, so the same math.
            double x = tag.rawPose.x, y = tag.rawPose.y, z = tag.rawPose.z;
            double behind = CAMERA_BACK + z * Math.cos(pitch) + y * Math.sin(pitch);
            double up = CAMERA_HEIGHT + z * Math.sin(pitch) - y * Math.cos(pitch);
            boolean use = z > 0 && behind <= 90;  // no 3D position, or too far to trust: not used
            // Everything the tag finder said about this tag, and what Auto5 made of it.
            logRow("tag", String.format(Locale.US, "id %d %s at pixel %.0f %.0f, margin %.0f, hamming %d;"
                            + " camera x %.2f y %.2f z %.2f in.; range %.1f in. bearing %.1f elevation %.1f yaw %.1f deg;"
                            + " -> behind %.1f right %.1f up %.1f in.%s",
                    id, CELLS[(id - 30) / 4], one.center.x, one.center.y, one.decisionMargin, one.hamming,
                    x, y, z, tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation, tag.ftcPose.yaw,
                    behind, -x, up, use ? "" : " NOT USED (too far / behind the camera)"));
            if (!use) {
                continue;
            }
            tagSeen[id - 30][0] += behind;
            tagSeen[id - 30][1] -= x;  // the camera looks backward: its right is the robot's left
            tagSeen[id - 30][2] += up;
            tagSeen[id - 30][3]++;
        }
    }

    /** Once the webcam is streaming: sends EXPOSURE_MS and GAIN (once). Says on the screen if it can't. */
    void setExposure() {
        if (exposureSet || portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        exposureSet = true;
        if (EXPOSURE_MS <= 0) {
            return;  // the webcam's own automatic exposure
        }
        try {
            ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            boolean ok = exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gain = portal.getCameraControl(GainControl.class);
            ok &= gain.setGain(Range.clip(GAIN, gain.getMinGain(), gain.getMaxGain()));
            if (!ok) {
                cameraSetup = "the webcam did not take EXPOSURE_MS / GAIN";
            }
        } catch (Exception e) {
            cameraSetup = "could not set the webcam's exposure: it stays on automatic";
        }
    }

    /** One CELL from tagSeen: where its middle is (Auto3's line through the tags it saw) and how high. */
    CellLook cellLook(int cell) {
        CellLook c = new CellLook();
        double place = 0;
        for (int t = 0; t < 4; t++) {
            double[] seen = tagSeen[cell * 4 + t];
            if (seen[3] > 0) {
                c.tags++;
                place += TAG_PLACE[t];
                c.behind += seen[0] / seen[3];
                c.right += seen[1] / seen[3];
                c.up += seen[2] / seen[3];
            }
        }
        if (c.tags == 0) {
            return c;
        }
        place /= c.tags;
        c.behind /= c.tags;
        c.right /= c.tags;
        c.up /= c.tags;
        double spread = 0, behindSlope = 0, rightSlope = 0;
        for (int t = 0; t < 4; t++) {
            double[] seen = tagSeen[cell * 4 + t];
            if (seen[3] > 0) {
                double along = TAG_PLACE[t] - place;
                spread += along * along;
                behindSlope += along * (seen[0] / seen[3] - c.behind);
                rightSlope += along * (seen[1] / seen[3] - c.right);
            }
        }
        if (spread > 0) {  // 2 or more tags: back along their line to the middle of the CELL
            c.behind -= behindSlope / spread * place;
            c.right -= rightSlope / spread * place;
        }
        return c;
    }

    /**
     * Is the CELL straight behind (which: UP_NEAR or UP_FAR) up, from the last look? Answers which CELL
     * is up (UP_NEAR / UP_FAR) or UP_UNKNOWN, and sets upNote. reference = how high its tags were when
     * it was up (see TIP_MOVE); NaN = not known yet, then only both CELLs in view can tell (whichIsUp).
     */
    int behindState(int which, double reference) {
        int other = which == UP_NEAR ? UP_FAR : UP_NEAR;
        String name = which == UP_NEAR ? "near" : "far";
        if (pictures < MIN_PICTURES) {
            upNote = "only " + pictures + " pictures from the webcam -> " + UP_NAMES[UP_UNKNOWN];
            return UP_UNKNOWN;
        }
        if (Double.isNaN(reference)) {
            return whichIsUp();  // nothing to compare with yet
        }
        CellLook c = cellLook(which == UP_NEAR ? nearCellIndex() : farCellIndex());
        if (c.tags == 0) {
            upNote = String.format(Locale.US, "%s CELL: its tags are GONE (%d pictures; up it showed them at %.1f in.) -> %s",
                    name, pictures, reference, UP_NAMES[other]);
            return other;
        }
        double moved = c.up - reference;
        int answer = Math.abs(moved) <= TIP_MOVE ? which : other;
        upNote = String.format(Locale.US, "%s CELL: %d tags at %.1f in. high, %+.1f from %.1f when up -> %s",
                name, c.tags, c.up, moved, reference, UP_NAMES[answer]);
        return answer;
    }

    /** Which CELL of our HIVE is up, from the last look. Sets upNote for the screen. */
    int whichIsUp() {
        CellLook n = cellLook(nearCellIndex()), f = cellLook(farCellIndex());
        String seen = String.format("near %d tags %s | far %d tags %s", n.tags, n.tags > 0 ? String.format("%.1f in. up", n.up) : "",
                f.tags, f.tags > 0 ? String.format("%.1f in. up", f.up) : "");
        int answer = UP_UNKNOWN;
        if (n.tags >= UP_MIN_TAGS && f.tags >= UP_MIN_TAGS) {
            double diff = n.up - f.up;
            answer = diff > UP_MARGIN ? UP_NEAR : diff < -UP_MARGIN ? UP_FAR : UP_UNKNOWN;
        } else if (ONE_CELL_SPLIT > 0 && (n.tags >= UP_MIN_TAGS || f.tags >= UP_MIN_TAGS)) {
            boolean nearOnly = n.tags >= UP_MIN_TAGS;
            double h = nearOnly ? n.up : f.up;
            if (h > ONE_CELL_SPLIT + UP_MARGIN / 2) {
                answer = nearOnly ? UP_NEAR : UP_FAR;
            } else if (h < ONE_CELL_SPLIT - UP_MARGIN / 2) {
                answer = nearOnly ? UP_FAR : UP_NEAR;
            }
        }
        String why = pictures == 0 ? "NO PICTURES from the webcam (" + portal.getCameraState() + "): plugged in? configured?"
                : n.tags + f.tags == 0 ? "no tags of our HIVE in view" : seen;
        upNote = why + " -> " + UP_NAMES[answer];
        return answer;
    }

    /**
     * Before a shot: turn so the launcher points at the middle of that CELL. Two answers are compared:
     * the camera's (where the CELL's tags are) and the odometry's (from where odometry says the robot
     * is to where the CELL is on the field). They agree within AIM_AGREE: trust the camera, it sees
     * the real CELL. They don't, or the camera saw fewer than 2 tags: trust odometry.
     */
    void aimAt(int which) {
        double[] cellXY = which == UP_NEAR ? NEAR_CELL_XY : FAR_CELL_XY;
        double odoOff = wrapDegrees(Math.toDegrees(Math.atan2(cellXY[1] - posY, cellXY[0] - posX)) + 180 - heading) + AIM_TRIM;
        CellLook c = cellLook(which == UP_NEAR ? nearCellIndex() : farCellIndex());
        double camOff = c.tags > 0 ? Math.toDegrees(Math.atan2(c.right, c.behind)) + AIM_TRIM : 999;  // + = turn LEFT
        double off;
        String from;
        if (c.tags >= 2 && c.behind <= FAR_CELL && Math.abs(camOff - odoOff) <= AIM_AGREE) {
            off = camOff;
            from = String.format("camera (odometry agrees: %+.1f)", odoOff);
        } else {
            off = odoOff;
            from = c.tags == 0 ? "odometry (no tags of that CELL)"
                    : c.tags < 2 ? String.format("odometry (camera 1 tag only: %+.1f)", camOff)
                    : String.format("odometry (camera said %+.1f: too different)", camOff);
        }
        if (Math.abs(off) <= AIM_CLOSE) {
            note(String.format("  aim: %.1f deg off, fine. From %s", Math.abs(off), from));
        } else if (!CAMERA_TURNS || Math.abs(off) > AIM_MAX) {
            note(String.format("  aim: %.1f deg off, no turn (%s). From %s", Math.abs(off),
                    !CAMERA_TURNS ? "CAMERA_TURNS off" : "too far off to be right", from));
        } else {
            turnTo(heading + off, AIM_TURN_TIME, true);  // its front is on a wall: see turnTo
            note(String.format("  aim: turned %.1f %s. From %s", Math.abs(off), off > 0 ? "LEFT" : "RIGHT", from));
        }
    }

    /**
     * INIT: do the tags match the alliance and the start spot that were picked? From the near-wall
     * spot the CELL straight behind should be our near CELL, from the far-wall spot our far CELL,
     * and from the side-wall spot both of ours side by side. It only warns: the picks stay.
     */
    String checkPicks() {
        int straight = -1;
        double nearest = Double.MAX_VALUE;
        int ours = 0, theirs = 0;
        for (int cell = 0; cell < 4; cell++) {
            CellLook c = cellLook(cell);
            boolean isOurs = cell == nearCellIndex() || cell == farCellIndex();
            if (isOurs) ours += c.tags; else theirs += c.tags;
            double off = c.tags > 0 ? Math.toDegrees(Math.atan2(c.right, c.behind)) : 999;
            if (c.tags > 0 && Math.abs(off) <= AIM_MAX && c.behind <= FAR_CELL && c.behind < nearest) {
                nearest = c.behind;
                straight = cell;
            }
        }
        if (pictures == 0) return "NO PICTURES from the webcam yet: without them it runs the route blind";
        if (ours + theirs == 0) return "no HIVE tags in view: it will run the route blind (or turn the robot to the HIVE)";
        String alliance = blue ? "BLUE" : "RED";
        if (theirs > ours) {
            return "WARNING: it sees mostly the " + (blue ? "RED" : "BLUE") + " HIVE. Is " + alliance + " right?";
        }
        if (startSpot == SIDE) {
            return "OK: our " + alliance + " HIVE in view (" + ours + " tags)";
        }
        int want = startSpot == NEAR ? nearCellIndex() : farCellIndex();
        if (straight == -1) return "no CELL straight behind: is the robot on the " + SPOT_NAMES[startSpot].toLowerCase() + "?";
        if (straight == want) {
            return String.format("OK: %s straight behind, %.0f in.: matches %s, %s", CELLS[straight], nearest, alliance, SPOT_NAMES[startSpot]);
        }
        boolean otherAlliance = straight != nearCellIndex() && straight != farCellIndex();
        return "WARNING: " + CELLS[straight] + " is straight behind. "
                + (otherAlliance ? "Is " + alliance + " right?" : "Is the start spot right?");
    }

    // ================================================================ ODOMETRY (Auto3's, in "our side" numbers)

    static double[] spotPose(int which) {
        return which == FAR ? FAR_SPOT : which == SIDE ? SIDE_SPOT : NEAR_SPOT;
    }

    /** The robot is here now: position, heading, an empty trail. */
    void startOdometry(double[] pose) {
        if (imu != null) {
            imu.resetYaw();
        }
        for (int i = 0; i < 4; i++) {
            startTicks[i] = lastTicks[i] = wheels[i].getCurrentPosition();
        }
        posX = planX = targetX = pose[0];
        posY = planY = targetY = pose[1];
        heading = startHeading = planHeading = wheelHeading = pose[2];
        offLine = 0;
        imuStuckInches = 0;
        wheelFixes = 0;
        been = new boolean[MAP_ROWS][MAP_COLS];
    }

    void updateOdometry() {
        double[] d = new double[4];
        for (int i = 0; i < 4; i++) {
            int ticks = wheels[i].getCurrentPosition();
            int moved = ticks - lastTicks[i];
            lastTicks[i] = ticks;
            evidence[i] += power[i] * moved;  // power and counts agreeing = the encoder counts the right way
            boolean known = Math.abs(evidence[i]) > 30;
            if (known) {
                encoderSign[i] = Math.signum(evidence[i]);
            }
            d[i] = (known || power[i] == 0 ? moved * encoderSign[i] : Math.abs(moved) * Math.signum(power[i])) * INCH_PER_TICK;
            countedTicks[i] += Math.abs(moved);
        }
        double total = countedTicks[0] + countedTicks[1] + countedTicks[2] + countedTicks[3];
        for (int i = 0; i < 4; i++) {
            double others = (total - countedTicks[i]) / 3;
            dead[i] = others > 100 && countedTicks[i] < 0.1 * others;
        }
        if (dead[0]) d[0] = d[2] + d[3] - d[1];
        if (dead[1]) d[1] = d[2] + d[3] - d[0];
        if (dead[2]) d[2] = d[0] + d[1] - d[3];
        if (dead[3]) d[3] = d[0] + d[1] - d[2];

        double forward = (d[0] + d[1] + d[2] + d[3]) / 4;
        double sideways = (d[0] - d[1] - d[2] + d[3]) / 4 * STRAFE_FACTOR;  // to the robot's right
        wheelHeading += Math.toDegrees((-d[0] + d[1] - d[2] + d[3]) / 4 / TURN_RADIUS);
        if (imu != null) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES);
            hubPitch = angles.getPitch(AngleUnit.DEGREES);
            hubRoll = angles.getRoll(AngleUnit.DEGREES);
            heading = startHeading + yaw;
            boolean hubOnEdge = !(Math.abs(hubPitch) <= HUB_EDGE);
            if (yaw == lastYaw && hubPitch == lastPitch && hubRoll == lastRoll) {
                imuStuckInches += Math.abs(forward) + Math.abs(sideways);
            } else {
                imuStuckInches = 0;
            }
            lastYaw = yaw;
            lastPitch = hubPitch;
            lastRoll = hubRoll;
            if (opModeIsActive()) {
                if (hubOnEdge) {
                    imu = null;
                    imuNote = "Hub isn't mounted the way HUB_LOGO / HUB_USB say: fix them. Heading from the wheels";
                } else if (imuStuckInches > IMU_STUCK) {
                    imu = null;
                    imuNote = "IMU has stopped working. Heading from the wheels";
                } else if (!turning && Math.abs(wrapDegrees(heading - wheelHeading)) > 25) {
                    wheelHeading = heading;
                    wheelFixes++;
                    imuNote = "wheels slipped / robot knocked " + wheelFixes + "x: IMU kept";
                }
            } else {
                imuNote = hubOnEdge ? "WARNING: the hub isn't mounted the way HUB_LOGO / HUB_USB say" : "";
            }
        }
        if (imu == null) {
            heading = wheelHeading;
        }
        double h = Math.toRadians(heading);
        posX += forward * Math.cos(h) + sideways * Math.sin(h);
        posY += forward * Math.sin(h) - sideways * Math.cos(h);
        int[] cell = mapCell(fieldX(blue, posX), fieldY(blue, posY));
        been[cell[0]][cell[1]] = true;
        rpmNow = rpm();  // read once per loop, for the screen and the log
        logRow("pos", "");
    }

    static double wrapDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    // ================================================================ SETUP (as Auto3)

    void setWheels(double forward, double right, double turn) {
        cmdForward = forward;
        cmdRight = right;
        cmdTurn = turn;
        double[] p = {forward + right + turn, forward - right - turn, forward - right + turn, forward + right - turn};
        double max = Math.max(1, Math.max(Math.max(Math.abs(p[0]), Math.abs(p[1])), Math.max(Math.abs(p[2]), Math.abs(p[3]))));
        for (int i = 0; i < 4; i++) {
            power[i] = p[i] / max;
            wheels[i].setPower(power[i]);
        }
    }

    void setUp() {
        wheels = new DcMotor[] {hardwareMap.get(DcMotor.class, "topleft"), hardwareMap.get(DcMotor.class, "topright"),
                hardwareMap.get(DcMotor.class, "bottomleft"), hardwareMap.get(DcMotor.class, "bottomright")};
        wheels[0].setDirection(DcMotorSimple.Direction.REVERSE);  // same as TeleOp
        wheels[2].setDirection(DcMotorSimple.Direction.REVERSE);
        wheels[3].setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor wheel : wheels) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        siphon = hardwareMap.get(CRServo.class, "servosiphon");
        siphon.setDirection(DcMotorSimple.Direction.REVERSE);
        // The flywheel is "turret" in the older configuration and "flywheel" in the newer one: either works.
        flywheel = hardwareMap.tryGet(DcMotorEx.class, "flywheel");
        if (flywheel == null) {
            flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        }
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(40, 0, 0, 32767 / (6000 * 28 / 60.0));

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        } else {
            imuNote = "NOT FOUND, heading from the wheels";
        }

        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, CAMERA_NAME);
        if (webcam == null) {
            cameraNote = "no \"" + CAMERA_NAME + "\" in the Robot Configuration: it runs the route blind";
            return;
        }
        // The 16 HIVE tags, each one on its own. (The SDK's own BIOBUZZ tag list joins each CELL's 4
        // tags into one and then never reports a single tag; cellLook wants every tag.)
        AprilTagLibrary.Builder hiveTags = new AprilTagLibrary.Builder();
        for (int id = 30; id <= 45; id++) {
            hiveTags.addTag(id, CELLS[(id - 30) / 4], TAG_SIZE, DistanceUnit.INCH);
        }
        tagFinder = new AprilTagProcessor.Builder()
                .setTagLibrary(hiveTags.build())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        tagFinder.setDecimation(TAG_DECIMATION);
        portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT_PX))
                .addProcessor(tagFinder)
                .build();  // the webcam starts up by itself during INIT; the Webcam line shows when it streams
        startLook();
    }

    double rpm() {
        return flywheel.getVelocity() * 60 / 28;
    }

    /** The webcam's state for the screen. */
    String webcamLine() {
        return String.format("%s, %.0f pictures a second, tag finder %s", portal.getCameraState(), portal.getFps(),
                portal.getProcessorEnabled(tagFinder) ? "on" : "off (driving)");
    }

    // ================================================================ THE PICKS

    /** INIT: B / Y alliance, dpad down / up start spot, X which CELL is up. Either gamepad. */
    void readPicks() {
        boolean moved = false;
        if (gamepad1.bWasPressed() | gamepad2.bWasPressed()) { blue = false; moved = true; }  // single | so both are read
        if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) { blue = true; moved = true; }
        if (gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed()) { startSpot = NEAR; moved = true; }
        if (gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed()) { startSpot = FAR; moved = true; }
        if (gamepad1.xWasPressed() | gamepad2.xWasPressed()) { upPick = (upPick + 1) % 3; }
        if (moved) {
            startOdometry(spotPose(startSpot));  // the robot is on the new spot now
        }
    }

    static String tile(boolean blue, int spot) {
        return spot == NEAR ? (blue ? "D6" : "C1") : spot == FAR ? (blue ? "D1" : "C6") : (blue ? "F3/F4" : "A3/A4");
    }

    static String wallName(boolean blue, int spot) {
        if (spot == SIDE) return blue ? "the BLUE wall" : "the RED wall";
        boolean audience = (spot == NEAR) != blue;  // red near = audience wall; blue far = audience wall
        return audience ? "the AUDIENCE wall" : "the wall OPPOSITE the audience";
    }

    // ================================================================ DEBUG SCREEN

    void showInit() {
        String alliance = blue ? "BLUE" : "RED";
        int up = startUp();
        String upWhy = upPick == PICK_NEAR || upPick == PICK_FAR ? "driver's pick (X)"
                : !CAMERA_DECIDES ? "CAMERA_DECIDES is off: normal setup"
                : cameraUp != UP_UNKNOWN ? "the camera sees it" : "camera can't tell: normal setup";
        telemetry.addLine("######## AUTO5 WEBCAM ########");
        telemetry.addLine(String.format(">> %s  |  %s %s  |  %s <<", alliance, SPOT_NAMES[startSpot], tile(blue, startSpot), UP_NAMES[up]));
        telemetry.addLine("   B red / Y blue    dpad: down NEAR spot, up FAR spot");
        telemetry.addLine("   X up CELL: " + (upPick == PICK_CAMERA ? "[CAMERA] near  far" : upPick == PICK_NEAR ? " camera [NEAR] far" : " camera  near [FAR]")
                + "   (" + upWhy + ")");
        telemetry.addLine("CHECK: robot on tile " + tile(blue, startSpot) + ", front on " + wallName(blue, startSpot)
                + ", back to the " + alliance + " HIVE");
        List<PlanStep> plan = planFor(startSpot, up);
        StringBuilder words = new StringBuilder();
        double total = 0;
        for (PlanStep s : plan) {
            words.append(words.length() > 0 ? " > " : "").append(s.word);
            total += s.seconds;
        }
        telemetry.addLine(String.format("PLAN: %s (about %.1f s)", words, total));
        telemetry.addData("Camera", portal == null ? cameraNote : initCheck);
        if (portal != null) {
            telemetry.addData("Webcam", webcamLine());
            telemetry.addData("Up", upNote);
        }
        if (!cameraSetup.isEmpty()) {
            telemetry.addLine("  " + cameraSetup);
        }
        if (!imuNote.isEmpty()) {
            telemetry.addData("IMU", imuNote);
        }
        showMap(plan, up);
        telemetry.addData("Heading", "%+5.1f deg (start %.0f)   hub tilt %+.0f %+.0f (flat floor: 0 0)", heading, startHeading, hubPitch, hubRoll);
        telemetry.update();
    }

    void show(String now) {
        telemetry.addLine(String.format(">> %s  |  started %s %s  |  up pick %s  |  %.1f s left <<", blue ? "BLUE" : "RED",
                SPOT_NAMES[startSpot], tile(blue, startSpot), upPick == PICK_NEAR ? "NEAR" : upPick == PICK_FAR ? "FAR" : "CAMERA", timeLeft()));
        telemetry.addData("Step", doing);
        telemetry.addData("Doing", now);
        telemetry.addData("Thinks", "%s (%s), holds %d, TIPS so far %d", UP_NAMES[upBelief], upFrom, held, tips);
        for (String n : lookNotes) {
            telemetry.addLine("  " + n);
        }
        showMap(null, upBelief);
        for (String s : stepsDone) {
            telemetry.addLine("  " + s);
        }
        telemetry.addData("Odometry", "X %5.1f  Y %5.1f  heading %+5.1f deg", posX, posY, wrapDegrees(heading));
        telemetry.addData("Target", "X %5.1f  Y %5.1f  off the line %+4.1f in", targetX, targetY, offLine);
        if (!fixNote.isEmpty()) {
            telemetry.addLine("  " + fixNote);
        }
        telemetry.addData("Heading", "IMU %s   wheels say %+5.1f deg   hub tilt %+.0f %+.0f",
                imu != null ? "in use" : "OFF", wrapDegrees(wheelHeading), hubPitch, hubRoll);
        if (!imuNote.isEmpty()) {
            telemetry.addLine("  " + imuNote);
        }
        String[] names = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            String status = dead[i] ? "NOT COUNTING" : Math.abs(evidence[i]) <= 30 ? "?" : encoderSign[i] < 0 ? "BACKWARD (fixed)" : "ok";
            telemetry.addData("Wheel " + names[i], "%7d ticks  power %+.2f  %s", lastTicks[i] - startTicks[i], power[i], status);
        }
        telemetry.addData("Flywheel", "%.0f of %.0f RPM", rpmNow, RPM);
        telemetry.addData("Webcam", portal == null ? cameraNote : webcamLine());
        telemetry.addData("Log", logName);
        telemetry.update();
    }

    // THE FIELD DRAWING: the whole field the way the AUDIENCE sees it (audience at the bottom, red
    // wall on the left, blue wall on the right), so it can be checked against the real field.
    // One column is 6 in. across, one row 12 in. deep (text is about twice as tall as it is wide).
    static final int MAP_ROWS = 12, MAP_COLS = 24;

    /** "Our side" X, Y to the field as the audience sees it: inches from the red wall, from the audience wall. */
    static double fieldX(boolean blue, double x) {
        return blue ? 144 - x : x;
    }

    static double fieldY(boolean blue, double y) {
        return blue ? 144 - y : y;
    }

    static int[] mapCell(double fieldX, double fieldY) {
        int row = (int) Math.floor((144 - fieldY) / 12);
        int col = (int) Math.floor(fieldX / 6);
        return new int[] {Range.clip(row, 0, MAP_ROWS - 1), Range.clip(col, 0, MAP_COLS - 1)};
    }

    static void mark(char[][] map, boolean blue, double x, double y, char symbol) {
        int[] cell = mapCell(fieldX(blue, x), fieldY(blue, y));
        map[cell[0]][cell[1]] = symbol;
    }

    void showMap(List<PlanStep> plan, int up) {
        for (String line : mapLines(blue, plan, plan == null ? been : null, up, posX, posY, heading)) {
            telemetry.addLine(line);
        }
    }

    /**
     * The drawing, as lines of text. plan = the planned route to draw (INIT); been = the trail driven
     * so far (during AUTO). up = which of our CELLs to show up. The robot is at x, y, facing h.
     */
    static List<String> mapLines(boolean blue, List<PlanStep> plan, boolean[][] been, int up, double x, double y, double h) {
        char[][] map = new char[MAP_ROWS][MAP_COLS];
        for (char[] row : map) {
            java.util.Arrays.fill(row, ' ');
        }
        if (plan != null) {
            for (PlanStep s : plan) {
                for (double[][] path : s.paths) {
                    for (int i = 1; i < path.length; i++) {
                        for (double t = 0; t <= 1; t += 0.05) {
                            mark(map, blue, path[i - 1][0] + (path[i][0] - path[i - 1][0]) * t,
                                    Math.max(1, Math.min(143, path[i - 1][1] + (path[i][1] - path[i - 1][1]) * t)), '.');
                        }
                    }
                }
            }
        }
        if (been != null) {
            for (int r = 0; r < MAP_ROWS; r++) {
                for (int c = 0; c < MAP_COLS; c++) {
                    if (been[r][c]) map[r][c] = 'o';
                }
            }
        }
        // Both HIVEs: ours U = up, d = down, ? = can't tell; theirs h.
        for (double dx = -9; dx <= 9; dx += 6) {
            mark(map, blue, NEAR_CELL_XY[0] + dx, NEAR_CELL_XY[1], up == UP_UNKNOWN ? '?' : up == UP_NEAR ? 'U' : 'd');
            mark(map, blue, FAR_CELL_XY[0] + dx, FAR_CELL_XY[1], up == UP_UNKNOWN ? '?' : up == UP_FAR ? 'U' : 'd');
            mark(map, blue, 144 - NEAR_CELL_XY[0] + dx, NEAR_CELL_XY[1], 'h');
            mark(map, blue, 144 - NEAR_CELL_XY[0] + dx, FAR_CELL_XY[1], 'h');
        }
        mark(map, blue, 10, 2, 'G');                       // our GARDEN corner
        mark(map, blue, PARK_SPOT[0], PARK_SPOT[1], 'L');  // where it parks (LOADING ZONE guess)
        mark(map, blue, NEAR_SPOT[0], NEAR_SPOT[1], 'N');
        mark(map, blue, FAR_SPOT[0], FAR_SPOT[1], 'F');
        // The robot: an arrow pointing where its FRONT (intake) faces, as the audience sees it.
        double a = wrapDegrees(h + (blue ? 180 : 0));  // 90 = toward the top of the drawing
        mark(map, blue, x, y, Math.abs(a) <= 45 ? '>' : Math.abs(a) >= 135 ? '<' : a > 0 ? '^' : 'v');
        List<String> lines = new ArrayList<>();
        StringBuilder top = new StringBuilder(" +");
        StringBuilder bottom = new StringBuilder(" +");
        for (int c = 0; c < MAP_COLS; c++) {
            top.append('-');
            bottom.append('-');
        }
        String topText = "WALL OPPOSITE AUDIENCE";
        top.replace(2, 2 + topText.length(), topText);
        String bottomText = "AUDIENCE";
        bottom.replace(10, 10 + bottomText.length(), bottomText);
        lines.add(top.append('+').toString());
        for (char[] row : map) {
            lines.add("R|" + new String(row) + "|B");
        }
        lines.add(bottom.append('+').toString());
        lines.add("R red wall  B blue wall  " + (blue ? "BLUE" : "RED") + " = us   N F start spots");
        lines.add("^ robot (points where the intake faces)  U/d our CELL up/down");
        lines.add("h their HIVE  G GARDEN  L park  " + (plan != null ? ". route" : "o driven"));
        return lines;
    }
}
