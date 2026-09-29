package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Driver-controlled mode for the whole robot: mecanum drive, intake and shooter.
 *
 * Gamepad 1 drives, and every mechanism control works from EITHER gamepad. One driver can run
 * the whole robot from gamepad 1, or a second driver can take the mechanisms on gamepad 2.
 *
 * Driving (gamepad 1):
 *   Left stick        move: forward, back, strafe left/right, or any diagonal
 *   Right stick X     turn left / right, in place or while moving
 *   Right bumper      hold for precision mode: slower, for lining up
 *   D-pad             nudge slowly forward / back / left / right, relative to the robot
 *                     (overrides the left stick)
 *   Back (Share)      switch between robot-centric and field-centric driving
 *   Start (Options)   reset heading: point the robot away from you, then press
 *
 * Mechanisms (either gamepad):
 *   Right trigger     hold to run the intake, pulling balls in
 *   Left trigger      hold to run the intake backwards, pushing balls out
 *   Y                 flywheel on / off
 *   A                 hold to fire: turns the carousel while the flywheel is at speed
 *   X                 hold to turn the carousel forward without firing, to move balls into place
 *   B                 hold to turn the carousel backward, to clear a jam
 *   LB + d-pad        hold the left bumper and press up / down to change the flywheel speed
 *                     by 100 RPM (on gamepad 1 the d-pad nudges when the bumper is not held)
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

    private MecanumDrivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private boolean fieldCentric = false;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        drivetrain.resetHeading();

        while (opModeInInit()) {
            handleDriveModeButtons();
            handleFlywheelSpeedButtons();
            // Button presses are remembered until read, so a Y press during INIT would otherwise
            // start the flywheel the moment the match starts. Read and ignore them.
            gamepad1.yWasPressed();
            gamepad2.yWasPressed();

            addStatusTelemetry();
            shooter.addTelemetry(telemetry);
            telemetry.addLine();
            telemetry.addLine("Left stick: move     Right stick: turn     Hold RB: precision");
            telemetry.addLine("D-pad: nudge    Back: robot/field-centric    Start: reset heading");
            telemetry.addLine("RT/LT: intake in/out    Y: flywheel on/off    A: fire");
            telemetry.addLine("X/B: carousel forward/back    LB + d-pad: flywheel speed");
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

            drive();
            runIntake();
            boolean waitingForFlywheel = runCarousel();

            addStatusTelemetry();
            shooter.addTelemetry(telemetry);
            if (waitingForFlywheel) {
                telemetry.addLine("Fire: waiting for the flywheel to reach speed");
            }
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

    /** Right trigger pulls balls in, left trigger pushes them out. In wins if both are held. */
    private void runIntake() {
        if (gamepad1.right_trigger_pressed || gamepad2.right_trigger_pressed) {
            intake.in();
        } else if (gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed) {
            intake.out();
        } else {
            intake.stop();
        }
    }

    /**
     * B turns the carousel backward, X turns it forward, and A fires: forward only while the
     * flywheel is at speed, so a ball is never pushed into a wheel that is too slow to launch it.
     * Returns true when A is held but the flywheel is not ready yet.
     */
    private boolean runCarousel() {
        boolean fire = gamepad1.a || gamepad2.a;
        boolean forward = gamepad1.x || gamepad2.x;
        boolean reverse = gamepad1.b || gamepad2.b;
        boolean ready = shooter.isReady();

        if (reverse) {
            shooter.feedReverse();
        } else if (forward || (fire && ready)) {
            shooter.feed();
        } else {
            shooter.stopFeeding();
        }
        return fire && !ready && !reverse;
    }

    /**
     * Back switches robot-/field-centric and Start resets the heading; a rumble confirms each.
     * Called every loop in INIT too, so presses made before START don't fire during the match.
     */
    private void handleDriveModeButtons() {
        if (gamepad1.backWasPressed() && drivetrain.hasImu()) {
            fieldCentric = !fieldCentric;
            gamepad1.rumbleBlips(fieldCentric ? 2 : 1);  // 2 blips = field-centric, 1 = robot-centric
        }
        if (gamepad1.startWasPressed() && drivetrain.hasImu()) {
            drivetrain.resetHeading();
            gamepad1.rumble(200);
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
        } else {
            telemetry.addData("Heading", "No IMU found, field-centric unavailable");
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
