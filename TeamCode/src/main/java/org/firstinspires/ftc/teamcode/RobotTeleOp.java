package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * goBILDA mecanum StarterBot (BIOBUZZ). The intake, the siphon and the flywheel spin for the
 * whole match. Left stick drives and strafes, right stick turns. Either gamepad works.
 *
 * The flywheel is held at a target RPM by Shooter instead of being run at flat full power, so
 * shots stay the same as the battery drains:
 *
 * AUTO (the default): DriveOdometry tracks where the robot is from its drive wheels, and the
 * flywheel speed is looked up from how far it is from the goal.
 *   before START: d-pad left / right   pick the starting spot, so it knows where the goal is
 *   d-pad up / down                    trim the RPM, in steps of Shooter.RPM_STEP (10 rpm)
 *   right bumper                       "the robot is back on its starting spot": resets the
 *                                      position, which drifts as the wheels slip
 *   left bumper                        switch between AUTO and MANUAL
 * MANUAL, for when the position has drifted too far to trust:
 *   d-pad left / right                 pick the shooting area (CLOSE, MID, FAR)
 *   d-pad up / down                    trim the RPM from there
 * Telemetry shows the position, the distance to the goal, the target and measured speed, and
 * READY once the flywheel is close enough to shoot.
 *
 * Wheel debugging: hold one button and only that wheel turns, positive power:
 *   triangle (Y) = front left     circle (B) = front right
 *   square (X)   = back left      cross (A)  = back right
 *
 * Robot Configuration names:
 *   topleft, topright, bottomleft, bottomright   wheel motors, Control Hub motor ports 0-3
 *   leftfeeder                                   intake servo, Control Hub servo 0
 *   mainmotorintake                              intake motor, Expansion Hub motor 0
 *   turret                                       flywheel motor, Expansion Hub motor 1
 *   rightfeeder                                  intake servo, Expansion Hub servo 0
 *   servosiphon                                  siphon servo, Expansion Hub servo 1
 */
@TeleOp(name = "Robot TeleOp", group = "Robot")
public class RobotTeleOp extends LinearOpMode {

    // Wheel directions, from the wheel test on 2026-10-05: with everything FORWARD only the front
    // right wheel rolled forward, so the other three are reversed here. Redo the test after any
    // rewiring: a wheel that rolls backward under its button needs the opposite setting.
    private static final DcMotorSimple.Direction TOP_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction TOP_RIGHT = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction BOTTOM_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction BOTTOM_RIGHT = DcMotorSimple.Direction.REVERSE;

    // How hard things spin, 0 to 1. The flywheel is set in RPM instead; see Shooter.
    private static final double INTAKE_POWER = 1.0;
    private static final double DEBUG_WHEEL_POWER = 0.3;  // one wheel at a time, button held

    // AUTO speed, part 1: where the goal is from each spot the robot can start on, in inches,
    // measured with a tape from the middle of the robot. "Forward" is the way the robot faces
    // when it is placed; a goal to its right has a negative "left". THESE ARE PLACEHOLDERS:
    // measure the real field. Add or remove starting spots, keeping the three lists in step.
    private static final String[] START_NAMES = {"START A", "START B"};
    private static final double[] GOAL_FORWARD_IN = {72.0, 72.0};
    private static final double[] GOAL_LEFT_IN = {24.0, -24.0};

    // AUTO speed, part 2: the flywheel RPM that scores from each distance to the goal, nearest
    // first. Distances in between are worked out on a straight line between the two nearest
    // rows; closer than the first or further than the last uses that row's RPM. THESE ARE
    // PLACEHOLDERS TOO: shoot from three or more measured distances, trim with d-pad up / down
    // until shots land, and write each distance and RPM in here.
    private static final double[] SHOT_DISTANCE_IN = {24.0, 60.0, 96.0};
    private static final double[] SHOT_RPM = {200.0, 250.0, 290.0};

    // MANUAL speed: one RPM for each shooting area, nearest the goal first. D-pad left / right
    // steps through them. THE RPM VALUES ARE PLACEHOLDERS: park the robot in each area, trim with d-pad
    // up / down until shots land, and write the number telemetry shows back in here. Add or
    // remove areas freely, as long as both lists stay the same length.
    private static final String[] AREA_NAMES = {"CLOSE", "MID", "FAR"};
    private static final double[] AREA_RPM = {200.0, 250.0, 290.0};
    private static final int STARTING_AREA = 1;  // MID

    @Override
    public void runOpMode() {
        DcMotor topLeft = hardwareMap.get(DcMotor.class, "topleft");
        DcMotor topRight = hardwareMap.get(DcMotor.class, "topright");
        DcMotor bottomLeft = hardwareMap.get(DcMotor.class, "bottomleft");
        DcMotor bottomRight = hardwareMap.get(DcMotor.class, "bottomright");
        DcMotor intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");

        // Owns the flywheel ("turret") and the siphon ("servosiphon"), and their directions.
        Shooter shooter = new Shooter(hardwareMap);

        topLeft.setDirection(TOP_LEFT);
        topRight.setDirection(TOP_RIGHT);
        bottomLeft.setDirection(BOTTOM_LEFT);
        bottomRight.setDirection(BOTTOM_RIGHT);
        for (DcMotor wheel : new DcMotor[] {topLeft, topRight, bottomLeft, bottomRight}) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // stop quickly when released
        }
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);  // the two feeders face each other

        DriveOdometry odometry = new DriveOdometry(hardwareMap, topLeft, topRight, bottomLeft, bottomRight);

        // Before the match: pick the spot the robot is starting on, so it knows where the goal is.
        int start = 0;
        while (opModeInInit()) {
            if (gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed()) {
                start = (start + 1) % START_NAMES.length;
            }
            if (gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed()) {
                start = (start + START_NAMES.length - 1) % START_NAMES.length;
            }
            telemetry.addLine("Ready. Intake, siphon and flywheel start when the match starts.");
            telemetry.addData("Starting spot", "%s   (d-pad left/right to change)", START_NAMES[start]);
            telemetry.addData("Goal from here", "%.0f in forward, %.0f in left",
                    GOAL_FORWARD_IN[start], GOAL_LEFT_IN[start]);
            if (!odometry.hasGyro()) {
                telemetry.addLine("No \"imu\" in the Robot Configuration: position will drift quickly");
            }
            telemetry.update();
        }
        double goalX = GOAL_FORWARD_IN[start];
        double goalY = GOAL_LEFT_IN[start];
        odometry.reset();  // the robot is on its starting spot now

        boolean autoSpeed = true;
        int area = STARTING_AREA;
        double trim = 0.0;  // rpm added by the driver with d-pad up / down

        // Everything that just spins.
        intake.setPower(INTAKE_POWER);
        leftFeeder.setPower(INTAKE_POWER);
        rightFeeder.setPower(INTAKE_POWER);
        shooter.feed();
        shooter.startFlywheel();

        while (opModeIsActive()) {
            double fl, fr, bl, br;

            odometry.update();
            double distance = odometry.distanceTo(goalX, goalY);

            // Single | on purpose, not ||: these WasPressed methods clear the button latch when
            // they are called, so short-circuiting would leave the other gamepad's press stored
            // up to fire a loop later, after the driver let go. Every button is read every loop
            // for the same reason, even the ones that do nothing in the current mode.
            boolean areaFurther = gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed();
            boolean areaCloser = gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed();
            boolean faster = gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed();
            boolean slower = gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed();
            boolean switchMode = gamepad1.leftBumperWasPressed() | gamepad2.leftBumperWasPressed();
            boolean backOnStart = gamepad1.rightBumperWasPressed() | gamepad2.rightBumperWasPressed();

            if (switchMode) {
                autoSpeed = !autoSpeed;
                trim = 0.0;
            }
            if (backOnStart) {
                odometry.reset();
            }
            if (!autoSpeed && (areaFurther || areaCloser)) {
                area = areaFurther ? Math.min(area + 1, AREA_RPM.length - 1) : Math.max(area - 1, 0);
                trim = 0.0;
            }
            if (faster) {
                trim += Shooter.RPM_STEP;
            }
            if (slower) {
                trim -= Shooter.RPM_STEP;
            }

            // Flywheel speed: from the distance to the goal in AUTO, from the chosen area in
            // MANUAL, plus the driver's trim either way. Only sent to the motor when it changes.
            double baseRpm = autoSpeed ? rpmForDistance(distance) : AREA_RPM[area];
            double wantedRpm = Math.max(0.0, Math.min(Shooter.MAX_RPM, Math.rint(baseRpm + trim)));
            if (wantedRpm != shooter.getTargetRpm()) {
                shooter.setTargetRpm(wantedRpm);
            }

            // Wheel debugging: hold triangle / circle / square / cross to run just that wheel.
            boolean testFrontLeft = gamepad1.y || gamepad2.y;    // triangle
            boolean testFrontRight = gamepad1.b || gamepad2.b;   // circle
            boolean testBackLeft = gamepad1.x || gamepad2.x;     // square
            boolean testBackRight = gamepad1.a || gamepad2.a;    // cross

            if (testFrontLeft || testFrontRight || testBackLeft || testBackRight) {
                fl = testFrontLeft ? DEBUG_WHEEL_POWER : 0.0;
                fr = testFrontRight ? DEBUG_WHEEL_POWER : 0.0;
                bl = testBackLeft ? DEBUG_WHEEL_POWER : 0.0;
                br = testBackRight ? DEBUG_WHEEL_POWER : 0.0;
                telemetry.addData("Wheel test", "%s%s%s%s", testFrontLeft ? "front left " : "",
                        testFrontRight ? "front right " : "", testBackLeft ? "back left " : "",
                        testBackRight ? "back right " : "");
            } else {
                // Whichever gamepad is pushed further drives. Stick Y reads negative when pushed forward.
                double forward = -stick(gamepad1.left_stick_y, gamepad2.left_stick_y);
                double strafe = stick(gamepad1.left_stick_x, gamepad2.left_stick_x);
                double turn = stick(gamepad1.right_stick_x, gamepad2.right_stick_x);

                // Mecanum: each wheel adds up what forward, strafe and turn ask of it (goBILDA / gm0).
                fl = forward + strafe + turn;
                fr = forward - strafe - turn;
                bl = forward - strafe + turn;
                br = forward + strafe - turn;

                // Keep every wheel within -1..1 without changing the ratios between them.
                double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
                if (max > 1.0) {
                    fl /= max;
                    fr /= max;
                    bl /= max;
                    br /= max;
                }
            }

            topLeft.setPower(fl);
            topRight.setPower(fr);
            bottomLeft.setPower(bl);
            bottomRight.setPower(br);

            telemetry.addData("Sticks", "GP1 L(%+.2f,%+.2f) R(%+.2f)   GP2 L(%+.2f,%+.2f) R(%+.2f)",
                    gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x,
                    gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);
            telemetry.addData("Wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", fl, fr, bl, br);
            if (autoSpeed) {
                telemetry.addData("Flywheel speed", "AUTO   %.0f in from goal   trim %+.0f rpm", distance, trim);
            } else {
                telemetry.addData("Flywheel speed", "MANUAL   area %s   trim %+.0f rpm", AREA_NAMES[area], trim);
            }
            shooter.addTelemetry(telemetry);
            odometry.addTelemetry(telemetry);
            telemetry.addLine("Left bumper = auto/manual   right bumper = robot is back on its start spot");
            telemetry.addLine("D-pad up/down = trim rpm   left/right = shooting area (manual)");
            telemetry.addLine("Hold triangle=front left  circle=front right  square=back left  cross=back right");
            telemetry.update();
        }
    }

    /** The flywheel RPM for a distance to the goal in inches, from the SHOT_ lists above. */
    private static double rpmForDistance(double distanceIn) {
        int last = SHOT_DISTANCE_IN.length - 1;
        if (distanceIn <= SHOT_DISTANCE_IN[0]) {
            return SHOT_RPM[0];
        }
        for (int i = 1; i <= last; i++) {
            if (distanceIn <= SHOT_DISTANCE_IN[i]) {
                double along = (distanceIn - SHOT_DISTANCE_IN[i - 1])
                        / (SHOT_DISTANCE_IN[i] - SHOT_DISTANCE_IN[i - 1]);
                return SHOT_RPM[i - 1] + along * (SHOT_RPM[i] - SHOT_RPM[i - 1]);
            }
        }
        return SHOT_RPM[last];
    }

    /** One stick axis from both gamepads: whichever is pushed further wins. */
    private static double stick(float pad1, float pad2) {
        return Math.abs(pad1) >= Math.abs(pad2) ? pad1 : pad2;
    }
}
