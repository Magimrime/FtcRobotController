package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Driver-controlled mode for a 4-wheel mecanum robot. The robot can move in any direction and
 * turn at the same time: forward/back, strafe left/right, diagonals, spinning in place, arcing
 * turns, and strafing while turning.
 *
 * Gamepad 1:
 *   Left stick        move: forward, back, strafe left/right, or any diagonal
 *   Right stick X     turn left / right, in place or while moving
 *   Right bumper      hold for precision mode: slower, for lining up
 *   D-pad             nudge slowly forward / back / left / right, relative to the robot
 *                     (overrides the left stick)
 *   Back (Share)      switch between robot-centric and field-centric driving
 *   Start (Options)   reset heading: point the robot away from you, then press
 *
 * Robot-centric: pushing the left stick forward drives the way the robot is facing.
 * Field-centric: pushing the left stick forward drives away from the driver, whichever way the
 *                robot is facing. This needs the IMU, so it is unavailable without one.
 *
 * The heading is zeroed at INIT, so "away from the driver" is the way the robot faces then.
 * Back and Start also work during INIT, so the mode and heading can be set before the match.
 */
@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumTeleOp extends LinearOpMode {

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
    private boolean fieldCentric = false;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap);
        drivetrain.resetHeading();

        while (opModeInInit()) {
            handleModeButtons();
            addStatusTelemetry();
            telemetry.addLine();
            telemetry.addLine("Left stick: move     Right stick: turn");
            telemetry.addLine("Hold RB: precision   D-pad: nudge");
            telemetry.addLine("Back: robot/field-centric   Start: reset heading");
            telemetry.update();
        }

        while (opModeIsActive()) {
            handleModeButtons();

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
            boolean nudging = gamepad1.dpad_up || gamepad1.dpad_down
                    || gamepad1.dpad_left || gamepad1.dpad_right;

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
            addStatusTelemetry();
            drivetrain.addTelemetry(telemetry);
            telemetry.update();
        }
    }

    /**
     * Back switches robot-/field-centric and Start resets the heading; a rumble confirms each.
     * Called every loop in INIT too: button presses are remembered until read, so a press made
     * before START would otherwise fire on the first loop of the match.
     */
    private void handleModeButtons() {
        if (gamepad1.backWasPressed() && drivetrain.hasImu()) {
            fieldCentric = !fieldCentric;
            gamepad1.rumbleBlips(fieldCentric ? 2 : 1);  // 2 blips = field-centric, 1 = robot-centric
        }
        if (gamepad1.startWasPressed() && drivetrain.hasImu()) {
            drivetrain.resetHeading();
            gamepad1.rumble(200);
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
