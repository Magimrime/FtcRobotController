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

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * Auto5: Auto3 with the LOGITECH C270 WEBCAM instead of the Limelight. Same route, same shots and
 * the same aim before each shot; only the camera is different. Auto3 stays as it is.
 *
 * START: backward in the middle of the wall behind our HIVE (blue: tile D6, red: tile C1), front
 * (intake) pressed against the wall, back (launcher) facing our upward CELL.
 *
 * PLAN (as Auto3): shoot 4, collect the GARDEN, slide home along the wall, drive to the other side
 * (GREEN or RED path, X switches it during INIT), turn around on the spot, press on the far wall,
 * shoot 4. Each shot is aimed with the camera first.
 *
 * CAMERA: the Logitech C270 webcam, named "Webcam 1" in the Robot Configuration. Plug it into a
 * Control Hub USB port, open the active configuration on the Driver Hub, press Scan, and save: it
 * shows up as "Webcam 1". It has to sit where the Limelight was: on the BACK next to the launcher,
 * looking the way it shoots. Somewhere else: change CAMERA_BACK and CAMERA_PITCH.
 * The Control Hub itself finds the AprilTags in the webcam picture (the SDK's AprilTag finder). It
 * uses 640x480, the only size the SDK has the C270's lens numbers for, so the distances come out
 * right. Finding tags costs the Control Hub time, so the finder only runs in INIT and while the
 * robot stands still to look before a shot; it is off while the robot drives.
 * Before each shot it looks at the 4 AprilTags under the CELL the robot is about to shoot at and
 * works out how far the launcher points off the middle of that CELL. More than AIM_CLOSE degrees
 * off: it turns onto the tags (by backing one side off the wall it is pressed on), looks again, then
 * shoots. The "Camera" line shows what it sees, and a "Shot 1" / "Shot 2" line stays up saying what
 * it did. No tags, no webcam, or a CELL that can't be the right one: it shoots the way it stands.
 * CHECK IN INIT: the "Webcam" line should say STREAMING and about 20 or more pictures a second. On
 * the start spot the Camera line should name our CELL (red: RED AUDIENCE, blue: BLUE SCORING),
 * about 53 in. behind, and an aim within a degree or two. If the robot is put down right and the
 * shots used to go in, but it says several degrees off, the camera isn't looking quite the way the
 * launcher shoots: change AIM_TRIM until it reads about 0 (it says "turn LEFT": make AIM_TRIM
 * smaller; "turn RIGHT": bigger). CAMERA_TURNS = false makes it only report. The Driver Hub's
 * Camera Stream (in INIT) shows the webcam picture with a box round every tag it finds.
 * Tags not found from the start spot: try TAG_DECIMATION = 1 (finds smaller tags, fewer pictures a
 * second). Picture too dark: raise EXPOSURE_MS or GAIN (0 = the webcam's own automatic exposure).
 *
 * BLOCKS: drivePath(spots, endsAtWall)  collectGarden()  turnAround()  aimWithCamera()  shoot()  spinSiphon(turns)  setIntake(power)
 * Every drivePath is ONE motion: full speed (slower only before a corner whose spot says so),
 * no stopping at the corners, never turning; odometry
 * corrects being pushed off the line or twisted inside the same wheel powers. The only turn is
 * turnAround() at the far side, standing still, so the launcher faces the HIVE (plus, before a
 * shot, a small turn onto the tags if the camera says the aim is off).
 *
 * TIME: as Auto3, about 13 s driving plus about 2.6 s per shot. Each look with the camera costs
 * about half a second (the first one while the flywheel speeds up anyway), and a turn onto the tags
 * up to about 2 s more.
 */
@Autonomous(name = "Auto5", group = "Robot", preselectTeleOp = "TeleOp")
public class Auto5 extends LinearOpMode {

    // THE ROUTE: spots {inches ahead of, inches right of} the start spot. Each list is driven as ONE
    // motion: straight lines through the spots, full speed, no stopping at the corners, never turning.
    // A spot can have a third number: the power to ARRIVE there with, to slow down before a sharp corner.
    static final double CORNER_POWER = 0.4;  // arriving at a sharp corner
    static final double CORNER_BACK = 21, CORNER_RIGHT = 30, GARDEN_RIGHT = 50, GARDEN_APPROACH = 10;
    static final double WALL_GAP = 3;  // how far it backs off the wall to slide home along it
    // To the GARDEN: back, right, straight to in front of the GARDEN corner, then forward into it with
    // the intake on. The last spot is past the wall on purpose: the wall stops it.
    static final double[][] TO_GARDEN = {{-CORNER_BACK, 0}, {-CORNER_BACK, CORNER_RIGHT},
            {-GARDEN_APPROACH, GARDEN_RIGHT}, {15, GARDEN_RIGHT}};
    // Home: a tiny bit off the wall, straight along it to the start line, forward into the start wall.
    static final double[][] HOME = {{-WALL_GAP, GARDEN_RIGHT}, {-WALL_GAP, 0}, {WALL_GAP + 3, 0}};

    // The two ways to the other side, as spots in inches: {ahead of, right of} the start spot.
    // The last spot is where it turns around: 14.5 in. from the far wall, 2 in. left of the start line.
    // GREEN: straight back on the start line, under the HIVE (it ends at -83 ahead), then from 9 in.
    // past it a slight bend (about 4 degrees) to the turn-around spot. That last line is 29 in.
    // long, so it has room to slow down instead of reaching the far wall at full speed.
    static final double[][] GREEN_PATH = {{-92, 0}, {-121, -2}};
    // RED: arrives at the two sharp corners near the far wall with CORNER_POWER, so it can make them.
    static final double[][] RED_PATH = {{-25, 0}, {-25, 25}, {-115.5, 25, CORNER_POWER},
            {-115.5, -2, CORNER_POWER}, {-121, -2}};
    static final double WALL_PUSH = 9;  // after turning: to the far wall (6 in. away) and 3 in. past it

    // Shooting. The siphon can't count its turns: if it does too many or too few, change SECONDS_PER_TURN.
    static final double RPM = 2800, SIPHON_POWER = 0.3, SECONDS_PER_TURN = 1.5;

    // THE CAMERA: the Logitech C270 webcam on the BACK next to the launcher, looking the way it
    // shoots (where the Limelight was). Before each shot it looks at the AprilTags under the CELL the
    // robot is about to shoot at, to see if the launcher really points at that CELL (see aimWithCamera).
    // CAMERA_BACK, CAMERA_PITCH, AIM_TRIM and the aim numbers mean the same as in Auto3.
    static final boolean CAMERA_TURNS = true;   // false = the camera only reports on the screen, the robot never turns to it
    static final String CAMERA_NAME = "Webcam 1";  // its name in the Robot Configuration (the name Scan gives it)
    static final int CAMERA_WIDTH = 640, CAMERA_HEIGHT = 480;  // the SDK has the C270's lens numbers only for 640x480
    // How small a tag it still finds: the picture is shrunk by this before the tags are searched.
    // 1 = not shrunk: finds the smallest / furthest tags, but the fewest pictures a second.
    // 2 = about twice the pictures. 3 = the SDK's normal setting, the fastest, the shortest range.
    static final float TAG_DECIMATION = 2;
    // A fixed exposure keeps the picture the same in a dark gym and a bright one, and a short one
    // doesn't blur. 0 = leave the webcam on its own automatic exposure. GAIN: 0 to 255 on the C270.
    static final int EXPOSURE_MS = 8, GAIN = 200;
    static final double TAG_SIZE = 3.25;         // inches, the black square of each HIVE tag
    static final double CAMERA_BACK = 8.0;       // inches the camera is behind the robot's middle
    static final double CAMERA_PITCH = 15.0;     // degrees it is tilted up (negative = down)
    static final double AIM_TRIM = 0.0;          // degrees. Shots land right of the CELL: make it +. Left: -.
    static final double AIM_CLOSE = 2.5;         // degrees: this close is aimed well enough, it doesn't turn
    static final double AIM_MAX = 10;            // degrees: a CELL further to the side than this is not the one it shoots at
    static final double FAR_CELL = 65;           // inches: a CELL further behind than this is not it either (ours is about 53)
    static final double LOOK_SETTLE = 0.15, LOOK_TIME = 0.4;  // seconds: stands still first, then looks this long
    static final double AIM_TURN_TIME = 1.5;     // seconds the turn onto the tags may take at most
    // The HIVE tags are ids 30-45: 4 in a row under each CELL, the CELLS in this order (from the SDK's tag list).
    static final String[] CELLS = {"RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING"};
    static final double[] TAG_PLACE = {-6.5, -2.75, 2.75, 6.5};  // inches along the row from the middle of the CELL

    // Driving: always full speed. Being pushed off the line or twisted is corrected inside the same
    // wheel powers while it keeps driving (see drivePath).
    static final double DRIVE_POWER = 1.0;
    static final double LINE_GAIN = 0.1, MAX_LINE_FIX = 0.5;     // sideways power per inch off the line
    static final double TWIST_GAIN = 0.02, MAX_TWIST_FIX = 0.3;  // turn power per degree twisted
    static final double SLOW_ZONE = 24, MIN_POWER = 0.1;         // slowing onto a last spot or before a sharp corner
    static final double SETTLE_TIME = 0.75;  // seconds within 1 in. of a last spot before it counts as there
    static final double TURN_CLOSE = 5, TURN_SETTLE = 1.0;  // a turn within 5 deg for 1 s is done (no hunting)
    static final double INCH_PER_TICK = Math.PI * 96 / 25.4 / 537.7;  // 96 mm wheels, 537.7 ticks/turn
    static final double TURN_RADIUS = (350 + 300) / 2.0 / 25.4;      // inches, for turning from the wheels

    DcMotor[] wheels;  // FL, FR, BL, BR
    DcMotor intake;
    CRServo leftFeeder, rightFeeder, siphon;
    DcMotorEx flywheel;
    IMU imu;
    String imuNote = "";
    // How the Control Hub is mounted on the robot. If the heading goes the wrong way when the robot
    // is turned by hand in INIT, change these to match the robot.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    boolean useRed = false;  // GREEN by default; X switches during INIT

    // Odometry: inches ahead of and right of the start spot, degrees turned left.
    // plan = where it should be and face after the last block; target / offLine are for the screen.
    double posAhead, posRight, heading, planAhead, planRight, planHeading, targetAhead, targetRight, offLine;
    final int[] startTicks = new int[4], lastTicks = new int[4];
    final double[] power = new double[4], evidence = new double[4], encoderSign = {1, 1, 1, 1};
    final double[] countedTicks = new double[4];
    final boolean[] dead = new boolean[4];
    double wheelHeading;  // heading worked out from the wheels alone, as a check on the IMU
    // The IMU is the heading. It only stops being trusted for one of two reasons (see updateOdometry):
    static final double HUB_EDGE = 70;    // hub tilt (pitch) past this = HUB_LOGO / HUB_USB are wrong
    static final double IMU_STUCK = 100;  // inches driven with the IMU giving exactly the same numbers = it stopped
    double hubPitch, hubRoll, lastYaw, lastPitch, lastRoll, imuStuckInches;
    int wheelFixes = 0;  // times the wheels' heading was wrong (slipped, knocked) and set back to the IMU's
    boolean turning = false;
    boolean[][] path = new boolean[MAP_ROWS][MAP_COLS];

    // The camera. portal == null: no webcam in the Robot Configuration, and everything works as before.
    VisionPortal portal;                  // the webcam's picture
    AprilTagProcessor tagFinder;          // finds the AprilTags in it
    boolean exposureSet = false;          // EXPOSURE_MS / GAIN sent (once the webcam streams)
    String cameraNote = "";               // what the camera saw last, for the screen
    String cameraSetup = "";              // a problem setting the webcam up, if any
    final String[] shotNotes = {"", ""};  // what the aim did before each shot
    int shotsAimed = 0;
    final double[][] tagSeen = new double[16][3];  // per tag 30-45: inches behind, inches right (added up), times seen
    int pictures;             // new pictures in this look
    String cellName = "";     // from the last look: the CELL the launcher points at,
    int cellTags;             // how many of its 4 tags were seen,
    double cellBehind, cellRight;  // where its middle is, in inches behind and right of the robot's middle,
    double aimOff;            // and the degrees the robot has to turn LEFT to point the launcher at it
    final ElapsedTime watch = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);  // so the map lines up
        setUp();
        startOdometry();
        while (opModeInInit()) {
            if (gamepad1.xWasPressed() | gamepad2.xWasPressed()) {  // single | so both presses are read
                useRed = !useRed;
            }
            updateOdometry();
            watchCell();
            show("INIT: X switches the path. Robot backward, front on the wall, middle tile");
        }
        startOdometry();  // the start spot is (0, 0), heading 0
        flywheel.setVelocity(RPM * 28 / 60.0);  // stays on the whole time
        findTags(false);  // the tag finder only runs while it looks (it slows the Control Hub)

        // ------------------------------------------------------------ THE PLAN
        shoot();
        collectGarden();           // one motion into the GARDEN, intake on for the last line
        drivePath(HOME, true);     // one motion home, the start wall stops it
        posAhead = planAhead = 0;  // pressed on the wall = exactly where it started
        drivePath(useRed ? RED_PATH : GREEN_PATH, false);  // one motion to the other side, easing onto the turn-around spot
        turnAround();  // the only turn, standing still: now the launcher faces the HIVE
        // Facing the other way, "forward" means ahead goes DOWN. Into the far wall, which stops it.
        drivePath(new double[][] {{planAhead - WALL_PUSH, planRight}}, true);
        shoot();

        while (opModeIsActive()) {  // keeps the debug screen up until AUTO ends
            updateOdometry();
            show("done");
        }
        if (portal != null) {
            portal.close();
        }
    }

    // ================================================================ BLOCKS

    /** ONE motion into the GARDEN, the intake coming on for the last line; the wall stops it. */
    void collectGarden() {
        drivePath(TO_GARDEN, true, TO_GARDEN.length - 1);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 0.25) {  // pressed on the wall: let the balls in
            updateOdometry();
            show("collecting");
        }
        setIntake(0);
        posAhead = planAhead = 0;  // pressed on the wall = exactly ahead 0, the same wall it started on
    }

    /**
     * ONE motion through these spots: straight lines, full speed, no stopping at the corners, and
     * it never turns. Every loop odometry (wheel encoders + IMU) says where the robot really is,
     * and the wheel powers come from three things added together:
     *   along the line    full speed toward the next spot
     *   off the line      pushed sideways? a push back onto the line (LINE_GAIN per inch)
     *   twisted           turned? a small turn back to the start direction (TWIST_GAIN per degree)
     * The along / off-the-line part is turned by the robot's real heading first (mecanum math), so
     * it keeps moving straight along the line even while a twist is being taken out.
     * endsAtWall = the last line runs full speed into a wall, which stops it (a sure position).
     * Otherwise it eases onto the last spot over SLOW_ZONE inches and stops there.
     * If something else stops it (another robot), the rest of the path is skipped and the next path
     * starts from where it really is.
     */
    void drivePath(double[][] spots, boolean endsAtWall) {
        drivePath(spots, endsAtWall, -1);
    }

    /** Same, with the intake coming on when line number intakeFromSpot starts (-1 = never). */
    void drivePath(double[][] spots, boolean endsAtWall, int intakeFromSpot) {
        for (int i = 0; i < spots.length; i++) {
            if (i == intakeFromSpot) {
                setIntake(1);
            }
            boolean last = i == spots.length - 1;
            double arrive = spots[i].length > 2 ? spots[i][2] : 1;  // third number: power to arrive with
            if (!driveLine(spots[i][0], spots[i][1], last && !endsAtWall, arrive)) {
                break;  // blocked
            }
        }
        setWheels(0, 0, 0);
    }

    /** One line of a path, arriving at its end with arrivePower (1 = full speed, less = slow for a corner). False if blocked. */
    boolean driveLine(double ahead, double right, boolean easeIn, double arrivePower) {
        targetAhead = ahead;
        targetRight = right;
        double fromAhead = planAhead;
        double fromRight = planRight;
        double length = Math.hypot(ahead - fromAhead, right - fromRight);
        double lineAhead = (ahead - fromAhead) / Math.max(length, 0.001);  // direction of the line
        double lineRight = (right - fromRight) / Math.max(length, 0.001);
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime stuck = new ElapsedTime();    // time since it last got a quarter inch further
        ElapsedTime settled = new ElapsedTime();  // time it has been on the last spot
        ElapsedTime close = new ElapsedTime();    // time it has been within 1 in. of the last spot
        double furthest = -Double.MAX_VALUE;
        boolean blocked = false;
        while (opModeIsActive() && timer.seconds() < 2 + length / 8) {
            updateOdometry();
            double movedAhead = posAhead - fromAhead;
            double movedRight = posRight - fromRight;
            double along = movedAhead * lineAhead + movedRight * lineRight;  // how far along the line
            offLine = -movedAhead * lineRight + movedRight * lineAhead;      // how far right of the line
            double toGo = length - along;                                     // negative = went past
            if (!easeIn && toGo < 1) {
                break;  // at the corner: straight on to the next line, the wheels keep going
            }
            if (easeIn && Math.hypot(toGo, offLine) < 0.5) {
                if (settled.seconds() > 0.1) {
                    break;  // on the spot and staying there
                }
            } else {
                settled.reset();
            }
            // Close but can't quite finish (the sideways push is too weak, or it stalls)? Don't wait
            // for the timeout: after SETTLE_TIME it counts as there, and the next move takes up the rest.
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
            if (easeIn) {  // only onto a last spot in open field: power in step with what is left
                speed = Range.clip(toGo / SLOW_ZONE, -1, 1) * DRIVE_POWER;  // backs up if it went past
                if (Math.abs(toGo) > 0.5 && Math.abs(speed) < MIN_POWER) {
                    speed = Math.copySign(MIN_POWER, toGo);
                }
            } else if (arrivePower < 1) {  // a sharp corner ahead: slow down to arrivePower over the last SLOW_ZONE inches
                speed = Math.max(arrivePower, Math.min(1, toGo / SLOW_ZONE)) * DRIVE_POWER;
            }
            // The three parts, in field directions (ahead / right of the start spot) ...
            double fix = Range.clip(LINE_GAIN * offLine, -MAX_LINE_FIX, MAX_LINE_FIX);
            double goAhead = lineAhead * speed + lineRight * fix;  // along the line + back onto it
            double goRight = lineRight * speed - lineAhead * fix;
            double twist = wrapDegrees(heading - planHeading);  // degrees, positive = turned too far left
            // ... turned into the robot's own forward / right by its real heading, plus the turn back.
            double h = Math.toRadians(heading);
            setWheels(goAhead * Math.cos(h) - goRight * Math.sin(h),
                      goAhead * Math.sin(h) + goRight * Math.cos(h),
                      Range.clip(TWIST_GAIN * twist, -MAX_TWIST_FIX, MAX_TWIST_FIX));
            show(String.format("driving to %.0f ahead, %.0f right", ahead, right));
        }
        planAhead = blocked ? posAhead : ahead;
        planRight = blocked ? posRight : right;
        return !blocked;
    }

    void turnAround() {
        turnTo(planHeading + 180, 5, false);
    }

    /**
     * Turns on the spot to a heading (degrees left of the start heading). It never drives while
     * turning. offTheWall = the front is pressed on a wall: turning on the spot would push a front
     * corner into the wall, so instead the side that has to go back backs off the wall while the
     * other side stands still.
     */
    void turnTo(double newHeading, double timeLimit, boolean offTheWall) {
        planHeading = newHeading;
        turning = true;
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        ElapsedTime close = null;  // starts the first time it gets within TURN_CLOSE degrees
        while (opModeIsActive() && timer.seconds() < timeLimit) {
            updateOdometry();
            double error = wrapDegrees(heading - planHeading);  // positive = turned too far left
            if (close == null && Math.abs(error) <= TURN_CLOSE) {
                close = new ElapsedTime();
            }
            if (close != null && close.seconds() > TURN_SETTLE) {
                break;  // nearly there but hunting: carry on, the next drive and the wall take out the rest
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
            show(offTheWall ? String.format("turning onto the tags, to %.1f deg", wrapDegrees(planHeading))
                            : String.format("turning on the spot to %.0f deg", wrapDegrees(planHeading)));
        }
        setWheels(0, 0, 0);
        turning = false;
        if (imu != null) {
            wheelHeading = heading;  // the wheels' guess drifts most while turning: line it up with the IMU
        }
    }

    /** Turn power (positive = clockwise) for a robot turned that many degrees too far left. */
    double turnPower(double degreesTooFarLeft) {
        double turn = Range.clip(0.01 * degreesTooFarLeft, -1.0, 1.0);
        return Math.abs(turn) < 0.2 ? Math.copySign(0.2, turn) : turn;
    }

    /**
     * Aims with the camera first (see aimWithCamera), and waits for the flywheel (2 s at most, the
     * aiming counted in), then the siphon: 1 turn forward (all 4 balls), a quarter turn backward, then
     * half a turn more forward to be safe. The intake helps push the balls up the whole time.
     */
    void shoot() {
        double pathHeading = planHeading;  // the aim may turn the robot a little; the next drive turns it back
        ElapsedTime timer = new ElapsedTime();
        aimWithCamera();  // while the flywheel gets up to speed
        while (opModeIsActive() && timer.seconds() < 2 && rpm() < RPM - 75) {
            updateOdometry();
            show("flywheel speeding up");
        }
        setIntake(1);
        spinSiphon(1);   // one turn shoots all 4 balls
        spinSiphon(-0.25);
        spinSiphon(0.5); // safety: makes sure every ball is out
        setIntake(0);
        planHeading = pathHeading;
    }

    /** Turns the siphon that many times: positive = forward (shooting), negative = backward. */
    void spinSiphon(double turns) {
        siphon.setPower(Math.signum(turns) * SIPHON_POWER);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < Math.abs(turns) * SECONDS_PER_TURN) {
            updateOdometry();
            show(String.format("siphon %s, about %.2f of %.2f turns", turns > 0 ? "forward" : "backward",
                    timer.seconds() / SECONDS_PER_TURN, Math.abs(turns)));
        }
        siphon.setPower(0);
    }

    void setIntake(double p) {
        intake.setPower(p);
        leftFeeder.setPower(p);
        rightFeeder.setPower(p);
    }

    // ================================================================ CAMERA

    /**
     * AIMS WITH THE CAMERA, just before a shot. The camera is on the back next to the launcher, so at
     * both shooting spots it looks at the CELL the robot is about to shoot at. It stands still, looks
     * at that CELL's AprilTags, and works out how many degrees the launcher points away from the
     * middle of the CELL. More than AIM_CLOSE off: it turns onto the tags and looks once more.
     * No webcam, no tags, only 1 tag, or CAMERA_TURNS off: it doesn't turn and shoots the way it
     * stands. What happened stays on the screen ("Shot 1", "Shot 2").
     */
    void aimWithCamera() {
        if (portal == null) {
            return;  // no camera: it shoots the way it stands
        }
        boolean found = lookAtCell("looking at the HIVE tags");
        String note = cameraNote;
        String why = found ? noTurnReason() : "shot not checked";
        if (why.isEmpty()) {
            double was = aimOff;
            turnTo(heading + aimOff, AIM_TURN_TIME, true);  // its front is on a wall: see turnTo
            lookAtCell("checking the aim");
            note = String.format("was %.1f deg off, turned %s. Now: %s", Math.abs(was), was > 0 ? "LEFT" : "RIGHT", cameraNote);
        } else {
            note += " -> " + why;
        }
        if (shotsAimed < shotNotes.length) {
            shotNotes[shotsAimed] = "Shot " + (shotsAimed + 1) + ": " + note;
        }
        shotsAimed++;
    }

    /** Why the robot won't turn onto the CELL just found, or "" if it will. */
    String noTurnReason() {
        return Math.abs(aimOff) <= AIM_CLOSE ? "fine"
                : !CAMERA_TURNS ? "no turn: CAMERA_TURNS is off"
                : cellTags < 2 ? "no turn: 1 tag is too rough to go by" : "";
    }

    /** INIT: twice a second the Camera line says what the camera sees, so the robot can be put down straight. */
    void watchCell() {
        if (portal == null) {
            return;
        }
        readTags();
        if (watch.seconds() > 0.5) {
            if (findCell()) {
                String why = noTurnReason();
                cameraNote += " -> " + (why.isEmpty() ? "it will turn " + (aimOff > 0 ? "LEFT" : "RIGHT") + " before the first shot" : why);
            }
            startLook();
            watch.reset();
        }
    }

    /** Stands still and looks at the HIVE tags for LOOK_TIME. True if it found the CELL the launcher points at. */
    boolean lookAtCell(String doing) {
        findTags(true);
        ElapsedTime timer = new ElapsedTime();
        boolean looking = false;
        while (opModeIsActive() && timer.seconds() < LOOK_SETTLE + LOOK_TIME) {
            updateOdometry();
            if (!looking && timer.seconds() >= LOOK_SETTLE) {
                startLook();  // pictures from before now were taken while the robot was still moving
                looking = true;
            }
            if (looking) {
                readTags();
            }
            show(doing);
        }
        findTags(false);
        return findCell();
    }

    /** Switches the tag finder on (to look) or off (while driving: it takes a lot of the Control Hub's time). */
    void findTags(boolean on) {
        if (portal != null) {
            portal.setProcessorEnabled(tagFinder, on);
        }
    }

    /** Starts a new look: forgets the tags counted so far; only pictures taken from now on count. */
    void startLook() {
        for (double[] tag : tagSeen) {
            tag[0] = tag[1] = tag[2] = 0;
        }
        pictures = 0;
        tagFinder.getFreshDetections();  // throws away the picture already waiting
    }

    /** Counts the tag finder's newest picture, if it is a new one, and adds the HIVE tags in it to tagSeen. */
    void readTags() {
        setExposure();
        List<AprilTagDetection> found = tagFinder.getFreshDetections();
        if (found == null) {
            return;  // no new picture since last time
        }
        pictures++;
        double pitch = Math.toRadians(CAMERA_PITCH);
        for (AprilTagDetection tag : found) {
            if (!(tag instanceof AprilTagSingleDetection) || tag.rawPose == null) {
                continue;
            }
            int id = ((AprilTagSingleDetection) tag).id;
            if (id < 30 || id > 45) {
                continue;  // not a HIVE tag
            }
            // Camera space: x right, y down, z straight out of the lens, in inches (the same as the
            // Limelight's in Auto3, so the same math).
            double x = tag.rawPose.x, y = tag.rawPose.y, z = tag.rawPose.z;
            double behind = CAMERA_BACK + z * Math.cos(pitch) + y * Math.sin(pitch);
            if (z <= 0 || behind > 90) {
                continue;  // no 3D position, or too far to trust
            }
            tagSeen[id - 30][0] += behind;
            tagSeen[id - 30][1] -= x;  // the camera looks backward: its right is the robot's left
            tagSeen[id - 30][2]++;
        }
    }

    /**
     * Works out from tagSeen where the middle of the CELL the launcher points at is. True if found.
     * A CELL's 4 tags are in a row, TAG_PLACE inches from its middle. Seeing only some of them (say
     * the two on one end), the middle of those is off to that side, so it follows the line through
     * the tags it did see back to place 0. One tag alone can't give that line: too rough to turn on.
     * The CELL it shoots at is the nearest one that is roughly straight behind (within AIM_MAX) and
     * no further than FAR_CELL; the other CELLS in view are further to the side or further away.
     */
    boolean findCell() {
        cellTags = 0;
        cellBehind = Double.MAX_VALUE;
        String rejected = "";
        for (int cell = 0; cell < 4; cell++) {
            int tags = 0;
            double place = 0, behind = 0, right = 0;  // averages over the tags of this CELL that were seen
            for (int t = 0; t < 4; t++) {
                double[] seen = tagSeen[cell * 4 + t];
                if (seen[2] > 0) {
                    tags++;
                    place += TAG_PLACE[t];
                    behind += seen[0] / seen[2];
                    right += seen[1] / seen[2];
                }
            }
            if (tags == 0) {
                continue;
            }
            place /= tags;
            behind /= tags;
            right /= tags;
            // The line through the tags: how much further behind / right it gets per inch along the row.
            double spread = 0, behindSlope = 0, rightSlope = 0;
            for (int t = 0; t < 4; t++) {
                double[] seen = tagSeen[cell * 4 + t];
                if (seen[2] > 0) {
                    double along = TAG_PLACE[t] - place;
                    spread += along * along;
                    behindSlope += along * (seen[0] / seen[2] - behind);
                    rightSlope += along * (seen[1] / seen[2] - right);
                }
            }
            if (spread > 0) {  // 2 or more tags: back along that line to place 0, the middle of the CELL
                behind -= behindSlope / spread * place;
                right -= rightSlope / spread * place;
            }
            double off = Math.toDegrees(Math.atan2(right, behind));
            if (Math.abs(off) > AIM_MAX || behind > FAR_CELL) {
                rejected = String.format("sees the %s CELL %.0f in. behind, %.0f deg to the side: not the one it shoots at",
                        CELLS[cell], behind, Math.abs(off));
            } else if (behind < cellBehind) {
                cellName = CELLS[cell];
                cellTags = tags;
                cellBehind = behind;
                cellRight = right;
            }
        }
        if (cellTags == 0) {
            cameraNote = pictures == 0 ? "NO PICTURES from the webcam (" + portal.getCameraState() + "): plugged in? configured?"
                    : rejected.isEmpty() ? "no HIVE tags in view" : rejected;
            return false;
        }
        // Positive = the CELL is to the robot's right (seen from above, front up), so it has to turn left.
        aimOff = Math.toDegrees(Math.atan2(cellRight, cellBehind)) + AIM_TRIM;
        cameraNote = String.format("%s CELL, %d of 4 tags, %.0f in. behind: aim %.1f deg off", cellName, cellTags, cellBehind, Math.abs(aimOff));
        return true;
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

    // ================================================================ ODOMETRY

    /** Makes where the robot is now the start spot: position (0, 0), heading 0, empty map path. */
    void startOdometry() {
        if (imu != null) {
            imu.resetYaw();
        }
        for (int i = 0; i < 4; i++) {
            startTicks[i] = lastTicks[i] = wheels[i].getCurrentPosition();
        }
        posAhead = posRight = heading = wheelHeading = 0;
        planAhead = planRight = planHeading = targetAhead = targetRight = offLine = 0;
        imuStuckInches = 0;
        wheelFixes = 0;
        path = new boolean[MAP_ROWS][MAP_COLS];
    }

    /** Adds how the robot moved since the last call to its position and heading. */
    void updateOdometry() {
        double[] d = new double[4];  // inches each wheel rolled
        for (int i = 0; i < 4; i++) {
            int ticks = wheels[i].getCurrentPosition();
            int moved = ticks - lastTicks[i];
            lastTicks[i] = ticks;
            // Power and counts agreeing = the encoder counts the right way; disagreeing = it counts
            // backward (the back right motor is wired the other way), so flip it. Until that is
            // known, assume the wheel turned the way it was driven.
            evidence[i] += power[i] * moved;
            boolean known = Math.abs(evidence[i]) > 30;
            if (known) {
                encoderSign[i] = Math.signum(evidence[i]);
            }
            d[i] = (known || power[i] == 0 ? moved * encoderSign[i] : Math.abs(moved) * Math.signum(power[i])) * INCH_PER_TICK;
            countedTicks[i] += Math.abs(moved);
        }
        // An encoder that hardly counts while the other wheels have rolled a couple of inches isn't
        // working (cable?).
        double total = countedTicks[0] + countedTicks[1] + countedTicks[2] + countedTicks[3];
        for (int i = 0; i < 4; i++) {
            double others = (total - countedTicks[i]) / 3;
            dead[i] = others > 100 && countedTicks[i] < 0.1 * others;
        }
        // A dead encoder's wheel is worked out from the other three: on a mecanum robot the two front
        // wheels always add up to the same as the two back wheels.
        if (dead[0]) d[0] = d[2] + d[3] - d[1];
        if (dead[1]) d[1] = d[2] + d[3] - d[0];
        if (dead[2]) d[2] = d[0] + d[1] - d[3];
        if (dead[3]) d[3] = d[0] + d[1] - d[2];

        double forward = (d[0] + d[1] + d[2] + d[3]) / 4;
        double sideways = (d[0] - d[1] - d[2] + d[3]) / 4;
        // Heading: the IMU. The wheels work it out too, but at full power and against walls they
        // slip, so when the two disagree by more than 25 degrees it is the wheels that are wrong:
        // their guess is set back to the IMU's and the robot straightens up to the IMU.
        // The IMU itself is only stopped being trusted (heading from the wheels from then on) when:
        //   the hub is on its edge   HUB_LOGO / HUB_USB don't match how the hub is mounted. Most wrong
        //                            settings still turn the right way, but some make the hub look
        //                            tilted up near 90 degrees (pitch), and then its heading is junk.
        //                            The screen shows the tilt: about 0 and 0 on a flat floor = right.
        //                            This is also warned about in INIT, before START.
        //   the IMU has stopped      exactly the same numbers while the robot drove IMU_STUCK inches.
        wheelHeading += Math.toDegrees((-d[0] + d[1] - d[2] + d[3]) / 4 / TURN_RADIUS);
        if (imu != null) {
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES);
            hubPitch = angles.getPitch(AngleUnit.DEGREES);
            hubRoll = angles.getRoll(AngleUnit.DEGREES);
            heading = yaw;
            boolean hubOnEdge = !(Math.abs(hubPitch) <= HUB_EDGE);  // right on the edge it can read NaN
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
            } else {  // INIT: warn before START, clear once it is fixed
                imuNote = hubOnEdge ? "WARNING: the hub isn't mounted the way HUB_LOGO / HUB_USB say. Fix them,"
                        + " or the heading will come from the wheels" : "";
            }
        }
        if (imu == null) {
            heading = wheelHeading;
        }
        double h = Math.toRadians(heading);
        posAhead += forward * Math.cos(h) + sideways * Math.sin(h);
        posRight += -forward * Math.sin(h) + sideways * Math.cos(h);
        int[] cell = mapCell(posAhead, posRight);
        path[cell[0]][cell[1]] = true;
    }

    static double wrapDegrees(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    // ================================================================ SETUP

    /** TeleOp's mecanum formula: forward, right and turn (clockwise) are each -1 to 1. */
    void setWheels(double forward, double right, double turn) {
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
        // The flywheel is "flywheel" in the newer configuration and "turret" in the older one: either works.
        flywheel = hardwareMap.tryGet(DcMotorEx.class, "flywheel");
        if (flywheel == null) {
            flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        }
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(40, 0, 0, 32767 / (6000 * 28 / 60.0));

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);  // all encoders read together, faster
        }
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        } else {
            imuNote = "NOT FOUND, heading from the wheels";
        }

        WebcamName webcam = hardwareMap.tryGet(WebcamName.class, CAMERA_NAME);
        if (webcam == null) {
            cameraNote = "no \"" + CAMERA_NAME + "\" in the Robot Configuration: the shots are not checked";
            return;
        }
        // The 16 HIVE tags, each one on its own. (The SDK's own BIOBUZZ tag list joins each CELL's 4
        // tags into one and then never reports a single tag; findCell wants every tag.)
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
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(tagFinder)
                .build();  // the webcam starts up by itself during INIT; the Webcam line shows when it streams
        startLook();
    }

    double rpm() {
        return flywheel.getVelocity() * 60 / 28;
    }

    // ================================================================ DEBUG SCREEN

    void show(String doing) {
        telemetry.addLine(useRed ? "=== PATH: RED (around the HIVE) ===   X switches before START"
                                 : "=== PATH: GREEN (straight, under the HIVE) ===   X switches before START");
        telemetry.addData("Doing", doing);
        telemetry.addData("Odometry", "%+6.1f in ahead %+6.1f in right   heading %+5.1f deg", posAhead, posRight, heading);
        telemetry.addData("Target", "%+6.1f in ahead %+6.1f in right   off the line %+4.1f in", targetAhead, targetRight, offLine);
        telemetry.addData("Heading", "IMU %s   wheels say %+5.1f deg   hub tilt %+.0f %+.0f (flat floor: 0 0)",
                imu != null ? "in use" : "OFF", wheelHeading, hubPitch, hubRoll);
        if (!imuNote.isEmpty()) {
            telemetry.addLine("  " + imuNote);
        }
        String[] names = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            String status = dead[i] ? "NOT COUNTING (worked out from the others)"
                    : Math.abs(evidence[i]) <= 30 ? "?" : encoderSign[i] < 0 ? "BACKWARD (fixed)" : "ok";
            telemetry.addData("Wheel " + names[i], "%7d ticks  power %+.2f  %s", lastTicks[i] - startTicks[i], power[i], status);
        }
        telemetry.addData("Flywheel", "%.0f of %.0f RPM", rpm(), RPM);
        if (portal != null) {
            telemetry.addData("Webcam", "%s, %.0f pictures a second, tag finder %s", portal.getCameraState(),
                    portal.getFps(), portal.getProcessorEnabled(tagFinder) ? "on" : "off (driving)");
        }
        telemetry.addData("Camera", cameraNote);
        if (!cameraSetup.isEmpty()) {
            telemetry.addLine("  " + cameraSetup);
        }
        for (String note : shotNotes) {
            if (!note.isEmpty()) {
                telemetry.addLine("  " + note);
            }
        }
        showMap();
        telemetry.update();
    }

    // The map: the whole field, seen from above with the wall the robot starts against at the top
    // and the far wall at the bottom. One column is 6 in. across, one row 12 in. deep (text is about
    // twice as tall as it is wide). The start is in the middle of the top wall.
    static final int MAP_ROWS = 12, MAP_COLS = 24;

    static int[] mapCell(double ahead, double right) {
        int row = (int) Math.floor((3 - ahead) / 12);
        int col = (int) Math.floor((right + 84) / 6);  // the start is 84 in. from the left wall
        return new int[] {Range.clip(row, 0, MAP_ROWS - 1), Range.clip(col, 0, MAP_COLS - 1)};
    }

    void showMap() {
        char[][] map = new char[MAP_ROWS][MAP_COLS];
        for (int r = 0; r < MAP_ROWS; r++) {
            for (int c = 0; c < MAP_COLS; c++) {
                map[r][c] = path[r][c] ? '.' : ' ';
            }
        }
        for (double a = -45; a >= -83; a -= 6) {  // the HIVE frame, in the middle of the field
            for (double r = -36; r <= 12; r += 3) {
                mark(map, a, r, 'H');
            }
        }
        // The chosen way to the other side, from the start spot to where it shoots.
        double[][] chosen = useRed ? RED_PATH : GREEN_PATH;
        double fromA = 0;
        double fromR = 0;
        for (int i = 0; i <= chosen.length; i++) {
            double toA = i < chosen.length ? chosen[i][0] : -127;  // last: pressed against the far wall
            double toR = i < chosen.length ? chosen[i][1] : chosen[chosen.length - 1][1];
            for (double t = 0; t <= 1; t += 0.02) {
                mark(map, fromA + (toA - fromA) * t, fromR + (toR - fromR) * t, useRed ? 'r' : 'g');
            }
            fromA = toA;
            fromR = toR;
        }
        mark(map, 0, 0, 'S');                // start
        mark(map, 0, GARDEN_RIGHT, 'G');     // GARDEN, in the corner
        mark(map, -127, chosen[chosen.length - 1][1], 'E');  // end: shoots from here
        double h = wrapDegrees(heading);     // robot, pointing the way its front faces
        mark(map, posAhead, posRight, Math.abs(h) <= 45 ? '^' : Math.abs(h) >= 135 ? 'v' : h > 0 ? '<' : '>');
        StringBuilder top = new StringBuilder("+");
        StringBuilder bottom = new StringBuilder("+");
        for (int c = 0; c < MAP_COLS; c++) {
            top.append('=');
            bottom.append('=');
        }
        telemetry.addLine(top.append("+ start wall").toString());
        for (char[] row : map) {
            telemetry.addLine("|" + new String(row) + "|");
        }
        telemetry.addLine(bottom.append("+ far wall").toString());
        telemetry.addLine("S start  G GARDEN  H HIVE  E shoots here  " + (useRed ? "r RED" : "g GREEN") + " path  . been  ^ robot");
    }

    static void mark(char[][] map, double ahead, double right, char symbol) {
        int[] cell = mapCell(ahead, right);
        map[cell[0]][cell[1]] = symbol;
    }
}
