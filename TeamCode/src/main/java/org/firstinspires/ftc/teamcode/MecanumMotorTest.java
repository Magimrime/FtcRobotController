package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

/*
 * Checks the drivetrain wiring before the first drive. It is listed in the Driver Station's
 * Utility menu. Prop the robot up so the wheels spin freely, then hold a button to run one
 * wheel forward at low power:
 *
 *   X (Square) = front left     Y (Triangle) = front right
 *   A (Cross)  = back left      B (Circle)   = back right
 *
 * Each wheel must turn the way that would roll the robot FORWARD.
 *   - The wrong wheel moves: fix the motor names or ports in the Robot Configuration.
 *   - The right wheel turns backward: flip its direction in MecanumDrivetrain.
 *
 * It also shows the port each wheel is configured on, which should match the diagram at the top
 * of MecanumDrivetrain. This goes through MecanumDrivetrain, so it tests the same names and motor
 * directions the TeleOp uses.
 */
@Utility(name = "Mecanum Motor Test", description = "Spin each drive wheel to check motor ports and directions")
public class MecanumMotorTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;

    @Override
    public void runOpMode() {
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);

        telemetry.addLine("Prop the robot up so the wheels are off the ground, then press START.");
        drivetrain.addPortTelemetry(telemetry);
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drivetrain.setWheelPowers(
                    gamepad1.x ? TEST_POWER : 0.0,
                    gamepad1.y ? TEST_POWER : 0.0,
                    gamepad1.a ? TEST_POWER : 0.0,
                    gamepad1.b ? TEST_POWER : 0.0);

            telemetry.addLine("Hold a button: that wheel must roll the robot FORWARD.");
            telemetry.addLine("X = front left     Y = front right");
            telemetry.addLine("A = back left      B = back right");
            telemetry.addLine();
            drivetrain.addPortTelemetry(telemetry);
            drivetrain.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
