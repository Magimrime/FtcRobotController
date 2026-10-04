package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleConsumer;

/*
 * Checks the robot's wiring one device at a time, before the first drive. It is listed in the
 * Driver Station's Utility menu (Driver Station app 11.2 or later).
 *
 * Prop the robot up so the wheels spin freely, then (either gamepad works):
 *   D-pad up / down   pick a device
 *   A (hold)          run it forward at low power
 *   B (hold)          run it backward
 *
 * The screen says what the device should do. If a different device moves, fix the names or
 * ports in the Robot Configuration. If the right device moves the wrong way, flip its direction
 * in MecanumDrivetrain, Intake or Shooter. The screen also shows the port each name is
 * configured on and the flywheel's motor model, so a configuration mistake shows up before
 * anything moves.
 *
 * Everything runs through the same classes the TeleOp uses, so it tests the same names and
 * directions.
 */
@Utility(name = "Robot Hardware Test", description = "Run each motor and servo one at a time to check ports and directions")
public class RobotHardwareTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;
    private static final double FLYWHEEL_TEST_RPM = 1000.0;

    /** One device to test: its name, what running it forward should do, and how to run it. */
    private static class Check {
        final String device;
        final String expected;
        final DoubleConsumer run;  // given +TEST_POWER (A), -TEST_POWER (B) or 0 every loop

        Check(String device, String expected, DoubleConsumer run) {
            this.device = device;
            this.expected = expected;
            this.run = run;
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        List<Check> checks = Arrays.asList(
                new Check("Front left wheel  (topleft)", "rolls the robot FORWARD",
                        p -> drivetrain.setWheelPowers(p, 0, 0, 0)),
                new Check("Front right wheel (topright)", "rolls the robot FORWARD",
                        p -> drivetrain.setWheelPowers(0, p, 0, 0)),
                new Check("Back left wheel   (bottomleft)", "rolls the robot FORWARD",
                        p -> drivetrain.setWheelPowers(0, 0, p, 0)),
                new Check("Back right wheel  (bottomright)", "rolls the robot FORWARD",
                        p -> drivetrain.setWheelPowers(0, 0, 0, p)),
                new Check("Intake motor (mainmotorintake)", "pulls a ball IN",
                        p -> intake.run(p, 0, 0)),
                new Check("Left feeder  (leftfeeder)", "pulls a ball IN",
                        p -> intake.run(0, p, 0)),
                new Check("Right feeder (rightfeeder)", "pulls a ball IN",
                        p -> intake.run(0, 0, p)),
                new Check("Siphon (servosiphon)", "turns the carousel TOWARD the flywheel",
                        shooter::runSiphon),
                new Check("Flywheel (turret)", "spins so a ball would launch OUT (A only; watch the rpm reading)",
                        p -> {
                            if (p > 0) {
                                shooter.setTargetRpm(FLYWHEEL_TEST_RPM);
                                shooter.startFlywheel();
                            } else {
                                shooter.stopFlywheel();
                            }
                        }));

        telemetry.addLine("Prop the robot up so the wheels are off the ground, then press START.");
        telemetry.addLine();
        drivetrain.addPortTelemetry(telemetry);
        intake.addPortTelemetry(telemetry);
        shooter.addPortTelemetry(telemetry);
        telemetry.update();
        waitForStart();

        int selected = 0;
        while (opModeIsActive()) {
            int previous = selected;
            if (gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed()) {
                selected = (selected + 1) % checks.size();
            }
            if (gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed()) {
                selected = (selected + checks.size() - 1) % checks.size();
            }
            if (selected != previous) {
                checks.get(previous).run.accept(0.0);  // stop the device we just left
            }

            boolean forward = gamepad1.a || gamepad2.a;   // either gamepad works
            boolean backward = gamepad1.b || gamepad2.b;
            double power = forward ? TEST_POWER : backward ? -TEST_POWER : 0.0;
            checks.get(selected).run.accept(power);

            telemetry.addLine("D-pad: pick a device.  Hold A: run forward.  Hold B: run backward.");
            telemetry.addData("Sticks", "GP1 (%+.2f,%+.2f)  GP2 (%+.2f,%+.2f)   0.00 while moving = pad not registered (Start+A / Start+B)",
                    gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad2.left_stick_x, -gamepad2.left_stick_y);
            for (int i = 0; i < checks.size(); i++) {
                telemetry.addLine((i == selected ? "> " : "   ") + checks.get(i).device);
            }
            telemetry.addLine();
            telemetry.addData("Forward should", checks.get(selected).expected);
            telemetry.addLine();
            drivetrain.addPortTelemetry(telemetry);
            intake.addPortTelemetry(telemetry);
            shooter.addPortTelemetry(telemetry);
            drivetrain.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            shooter.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
