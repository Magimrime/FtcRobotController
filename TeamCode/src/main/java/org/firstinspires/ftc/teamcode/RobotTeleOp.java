package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Driver-controlled mode for the whole robot: mecanum drive, intake and shooter.
 *
 * Gamepad 1 drives, and every mechanism control works from EITHER gamepad. One driver can run
 * the whole robot from gamepad 1, or a second driver can take the mechanisms on gamepad 2.
 *
 * Driving (gamepad 1):
 *   Left stick        move: forward, back, strafe left/right, or any diagonal. It never turns
 *                     the robot: the robot keeps facing the same way and goes in a straight line
 *   Right stick X     turn left / right, in place or while moving
 *   Right stick click straight-line assist on / off (see below)
 *   Right bumper      hold for precision mode: slower, for lining up
 *   D-pad             nudge slowly forward / back / left / right, relative to the robot
 *                     (overrides the left stick)
 *   Back (Share)      switch between robot-centric and field-centric driving
 *   Start (Options)   reset heading: point the robot away from you, then press
 *
 * Mechanisms (either gamepad):
 *   Right trigger     hold to run the intake motor, pulling balls in
 *   Left trigger      hold to run the intake backwards, pushing balls out
 *   Y                 flywheel on / off
 *   A                 siphon on / off: press once to start the carousel, again to stop it
 *   B                 hold to turn the carousel backward, to clear a jam
 *   LB + d-pad        hold the left bumper and press up / down to change the flywheel speed
 *                     by 100 RPM (on gamepad 1 the d-pad nudges when the bumper is not held)
 *
 * The two feeder servos at the intake run the whole match, pulling balls in. They only reverse
 * while the left trigger is pushing balls out.
 *
 * Straight-line assist: wheels never grip evenly, so a robot told to go straight slowly curves.
 * While the robot is moving and the right stick is centered, the assist uses the IMU to hold the
 * way the robot is facing, so it goes straight. It needs the IMU, and it is off while turning.
 *
 * Robot-centric: pushing the left stick forward drives the way the robot is facing.
 * Field-centric: pushing the left stick forward drives away from the driver, whichever way the
 *                robot is facing. This needs the IMU, so it is unavailable without one.
 *
 * The heading is zeroed at INIT, so "away from the driver" is the way the robot faces then.
 * Nothing moves during INIT, but the drive mode, heading and flywheel speed can be set then.
 */
@TeleOp(name = "Robot TeleOp", group = "Robot")
public class RobotTeleOp extends LinearOpMode {

    // Sticks closer to center than this count as centered, so worn sticks don't creep the robot.
    private static final double STICK_DEADBAND = 0.05;
    // Stick response: 1 = linear, 2 = squared, 3 = cubed. Higher gives finer control at low
    // speed while still reaching full speed at full stick.
    private static final double STICK_EXPONENT = 2.0;

    // Wheel power limits for each speed mode (1.0 = full power).
    private static final double NORMAL_MAX_POWER = 1.0;
    private static final double PRECISION_MAX_POWER = 0.4;
    private static final double NUDGE_POWER = 0.3;

    // Straight-line assist. The gain is the turn power used per degree the robot is off course:
    // raise it if the robot still curves, lower it if the robot wobbles side to side.
    private static final double HEADING_HOLD_GAIN = 0.02;
    // The most turn power the assist may use, so it can never out-muscle the driver.
    private static final double HEADING_HOLD_MAX_TURN = 0.3;
    // The robot keeps spinning for a moment after the turn stick is released. The assist waits
    // this long before locking the heading, so it doesn't swing back to where the stick let go.
    private static final double HEADING_HOLD_SETTLE_SECONDS = 0.3;

    private MecanumDrivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private boolean fieldCentric = false;
    private boolean siphonOn = false;

    private boolean straightAssist = true;
    private boolean holdingHeading = false;
    private double heldHeading;  // degrees, only meaningful while holdingHeading
    private final ElapsedTime sinceDriverTurned = new ElapsedTime();

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        drivetrain.resetHeading();

        while (opModeInInit()) {
            handleDriveModeButtons();
            handleFlywheelSpeedButtons();
            // Button presses are remembered until read, so a Y or A press during INIT would
            // otherwise start the flywheel or siphon the moment the match starts. Read and
            // ignore them.
            gamepad1.yWasPressed();
            gamepad2.yWasPressed();
            gamepad1.aWasPressed();
            gamepad2.aWasPressed();

            addStatusTelemetry();
            shooter.addTelemetry(telemetry);
            telemetry.addLine();
            telemetry.addLine("Left stick: move     Right stick: turn     Hold RB: precision");
            telemetry.addLine("D-pad: nudge    Back: robot/field-centric    Start: reset heading");
            telemetry.addLine("Right stick click: straight-line assist on/off");
            telemetry.addLine("RT/LT: intake in/out    Y: flywheel on/off    A: siphon on/off");
            telemetry.addLine("B: carousel backward    LB + d-pad: flywheel speed");
            telemetry.update();
        }

        while (opModeIsActive()) {
            handleDriveModeButtons();
            handleFlywheelSpeedButtons();

            // Read both gamepads every loop so neither one's press is left waiting.
            boolean y1 = gamepad1.yWasPressed();
            boolean y2 = gamepad2.yWasPressed();
            if (y1 || y2) {
                shooter.toggleFlywheel();
            }
            boolean a1 = gamepad1.aWasPressed();
            boolean a2 = gamepad2.aWasPressed();
            if (a1 || a2) {
                siphonOn = !siphonOn;
            }

            drive();
            runIntake();
            runCarousel();

            addStatusTelemetry();
            shooter.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            drivetrain.addTelemetry(telemetry);
            telemetry.update();
        }
    }

    /** Turns gamepad 1's sticks, bumper and d-pad into drivetrain motion. */
    private void drive() {
        // Stick Y reads negative when pushed forward, so flip it.
        double stickX = gamepad1.left_stick_x;
        double stickY = -gamepad1.left_stick_y;

        // Shape how far the stick is pushed, then point that amount in the stick's direction.
        // Shaping X and Y separately would bend diagonal moves toward the nearest axis.
        double stickDistance = Math.hypot(stickX, stickY);
        double moveAmount = shape(stickDistance);
        double forward = 0.0;
        double strafe = 0.0;
        if (moveAmount > 0.0) {
            forward = stickY / stickDistance * moveAmount;
            strafe = stickX / stickDistance * moveAmount;
        }
        double turn = shape(gamepad1.right_stick_x);

        boolean precision = gamepad1.right_bumper;
        // With the left bumper held, the d-pad sets the flywheel speed instead of nudging.
        boolean nudging = !gamepad1.left_bumper && (gamepad1.dpad_up || gamepad1.dpad_down
                || gamepad1.dpad_left || gamepad1.dpad_right);

        turn = holdHeading(turn, nudging || moveAmount > 0.0);

        if (nudging) {
            double nudgeForward = (gamepad1.dpad_up ? 1.0 : 0.0) - (gamepad1.dpad_down ? 1.0 : 0.0);
            double nudgeStrafe = (gamepad1.dpad_right ? 1.0 : 0.0) - (gamepad1.dpad_left ? 1.0 : 0.0);
            drivetrain.drive(nudgeForward, nudgeStrafe, turn, NUDGE_POWER);
        } else {
            double maxPower = precision ? PRECISION_MAX_POWER : NORMAL_MAX_POWER;
            if (fieldCentric) {
                drivetrain.driveFieldCentric(forward, strafe, turn, maxPower);
            } else {
                drivetrain.drive(forward, strafe, turn, maxPower);
            }
        }

        telemetry.addData("Speed", nudging ? "Nudge" : precision ? "Precision" : "Normal");
    }

    /**
     * Straight-line assist. Returns the turn to drive with: the driver's own while they are
     * turning, otherwise a correction that keeps the robot facing the way it was when it started
     * moving. The assist only works while the robot is being driven, so a parked robot that gets
     * bumped doesn't fight back or swing around when it next moves.
     */
    private double holdHeading(double driverTurn, boolean moving) {
        if (driverTurn != 0.0) {
            sinceDriverTurned.reset();
        }
        if (!straightAssist || !drivetrain.hasImu() || !moving
                || sinceDriverTurned.seconds() < HEADING_HOLD_SETTLE_SECONDS) {
            holdingHeading = false;
            return driverTurn;
        }

        double heading = drivetrain.getHeading(AngleUnit.DEGREES);
        if (!holdingHeading) {
            holdingHeading = true;
            heldHeading = heading;
        }
        // Heading counts up counter-clockwise but positive turn is clockwise, so a robot that has
        // drifted counter-clockwise (positive error) needs a positive turn to come back.
        double error = AngleUnit.normalizeDegrees(heading - heldHeading);
        return Range.clip(error * HEADING_HOLD_GAIN, -HEADING_HOLD_MAX_TURN, HEADING_HOLD_MAX_TURN);
    }

    /**
     * Right trigger pulls balls in, left trigger pushes them out. In wins if both are held. The
     * feeder servos keep pulling balls in when neither is held.
     */
    private void runIntake() {
        if (gamepad1.right_trigger_pressed || gamepad2.right_trigger_pressed) {
            intake.in();
        } else if (gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed) {
            intake.out();
        } else {
            intake.feedersOnly();
        }
    }

    /** A switches the siphon on and off. Holding B turns the carousel backward to clear a jam. */
    private void runCarousel() {
        if (gamepad1.b || gamepad2.b) {
            shooter.feedReverse();
        } else if (siphonOn) {
            shooter.feed();
        } else {
            shooter.stopFeeding();
        }
    }

    /**
     * Back switches robot-/field-centric, Start resets the heading and clicking the right stick
     * switches the straight-line assist; a rumble confirms each. Called every loop in INIT too,
     * so presses made before START don't fire during the match.
     */
    private void handleDriveModeButtons() {
        if (gamepad1.backWasPressed() && drivetrain.hasImu()) {
            fieldCentric = !fieldCentric;
            gamepad1.rumbleBlips(fieldCentric ? 2 : 1);  // 2 blips = field-centric, 1 = robot-centric
        }
        if (gamepad1.startWasPressed() && drivetrain.hasImu()) {
            drivetrain.resetHeading();
            holdingHeading = false;  // the held heading was measured from the old zero
            gamepad1.rumble(200);
        }
        if (gamepad1.rightStickButtonWasPressed() && drivetrain.hasImu()) {
            straightAssist = !straightAssist;
            gamepad1.rumbleBlips(straightAssist ? 2 : 1);  // 2 blips = on, 1 = off
        }
    }

    /** Left bumper + d-pad up / down raises or lowers the flywheel speed one step per press. */
    private void handleFlywheelSpeedButtons() {
        handleFlywheelSpeedButtons(gamepad1);
        handleFlywheelSpeedButtons(gamepad2);
    }

    private void handleFlywheelSpeedButtons(Gamepad gamepad) {
        // Always read the presses, so one made without the bumper isn't remembered until later.
        boolean up = gamepad.dpadUpWasPressed();
        boolean down = gamepad.dpadDownWasPressed();
        if (gamepad.left_bumper) {
            if (up) {
                shooter.adjustTargetRpm(Shooter.RPM_STEP);
            }
            if (down) {
                shooter.adjustTargetRpm(-Shooter.RPM_STEP);
            }
        }
    }

    private void addStatusTelemetry() {
        telemetry.addData("Drive", fieldCentric ? "Field-centric" : "Robot-centric");
        if (drivetrain.hasImu()) {
            telemetry.addData("Heading", "%.1f deg", drivetrain.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Straight-line assist",
                    !straightAssist ? "Off" : holdingHeading ? "Holding" : "On");
        } else {
            telemetry.addData("Heading", "No IMU found: no field-centric or straight-line assist");
        }
    }

    /**
     * Applies the deadband and response curve to a stick reading, keeping its sign. The output
     * rises smoothly from 0 at the edge of the deadband to 1 at full deflection.
     */
    private static double shape(double input) {
        double magnitude = Math.min(Math.abs(input), 1.0);
        if (magnitude < STICK_DEADBAND) {
            return 0.0;
        }
        double scaled = (magnitude - STICK_DEADBAND) / (1.0 - STICK_DEADBAND);
        return Math.copySign(Math.pow(scaled, STICK_EXPONENT), input);
    }
}
