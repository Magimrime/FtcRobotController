package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

/*
 * Checks the robot's wiring one device at a time, before the first drive. It is listed in the
 * Driver Station's Utility menu (Driver Station app 11.2 or later).
 *
 * Prop the robot up so the wheels spin freely, then (either gamepad works):
 *   D-pad up / down   pick a device
 *   A (hold)          run it forward at low power while held
 *   B (hold)          run it backward while held
 *   X                 toggle it ON forward; it keeps running while you pick other devices
 *   Y                 toggle it ON backward
 *   Back              stop everything
 *
 * So you can test one device at a time with A/B, or latch several on with X/Y and watch them
 * run together.
 *
 * The screen says what the device should do. If a different device moves, fix the names or
 * ports in the Robot Configuration. If the right device moves the wrong way, flip its direction
 * constant below (and the matching one in the TeleOp / MechanismTestAuto). The screen also shows
 * the port each name is configured on, so a configuration mistake shows up before anything moves.
 *
 * Robot Configuration names:
 *   topleft, topright, bottomleft, bottomright   wheel motors, Control Hub motor ports 0-3
 *   mainmotorintake                              intake motor, Expansion Hub motor 0
 *   turret                                       flywheel motor, Expansion Hub motor 1
 *   leftfeeder                                   intake servo, Control Hub servo 0
 *   rightfeeder                                  intake servo, Expansion Hub servo 0
 *   servosiphon                                  siphon servo, Expansion Hub servo 1
 */
@Utility(name = "Robot Hardware Test", description = "Run each motor and servo one at a time to check ports and directions")
public class RobotHardwareTest extends LinearOpMode {

    private static final double TEST_POWER = 0.3;

    // Directions, same as the TeleOp and MechanismTestAuto. Flip any that run backward.
    private static final DcMotorSimple.Direction TOP_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction TOP_RIGHT = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction BOTTOM_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction BOTTOM_RIGHT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction INTAKE = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction LEFT_FEEDER = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_FEEDER = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction SIPHON = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction FLYWHEEL = DcMotorSimple.Direction.FORWARD;

    /** One device to test: its label, what running it forward should do, and how to power it. */
    private static class Check {
        final String label;
        final String expected;
        final DcMotorSimple device;  // DcMotor and CRServo both have setPower()
        final int port;              // DcMotorSimple itself has no getPortNumber(), so it is passed in

        Check(String label, String expected, DcMotor motor) {
            this(label, expected, motor, motor.getPortNumber());
        }

        Check(String label, String expected, CRServo servo) {
            this(label, expected, servo, servo.getPortNumber());
        }

        private Check(String label, String expected, DcMotorSimple device, int port) {
            this.label = label;
            this.expected = expected;
            this.device = device;
            this.port = port;
        }
    }

    @Override
    public void runOpMode() {
        DcMotor topLeft = hardwareMap.get(DcMotor.class, "topleft");
        DcMotor topRight = hardwareMap.get(DcMotor.class, "topright");
        DcMotor bottomLeft = hardwareMap.get(DcMotor.class, "bottomleft");
        DcMotor bottomRight = hardwareMap.get(DcMotor.class, "bottomright");
        DcMotor intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        DcMotor flywheel = hardwareMap.get(DcMotor.class, "turret");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");
        CRServo siphon = hardwareMap.get(CRServo.class, "servosiphon");

        topLeft.setDirection(TOP_LEFT);
        topRight.setDirection(TOP_RIGHT);
        bottomLeft.setDirection(BOTTOM_LEFT);
        bottomRight.setDirection(BOTTOM_RIGHT);
        intake.setDirection(INTAKE);
        flywheel.setDirection(FLYWHEEL);
        leftFeeder.setDirection(LEFT_FEEDER);
        rightFeeder.setDirection(RIGHT_FEEDER);
        siphon.setDirection(SIPHON);

        List<Check> checks = Arrays.asList(
                new Check("Front left wheel  (topleft)", "rolls the robot FORWARD", topLeft),
                new Check("Front right wheel (topright)", "rolls the robot FORWARD", topRight),
                new Check("Back left wheel   (bottomleft)", "rolls the robot FORWARD", bottomLeft),
                new Check("Back right wheel  (bottomright)", "rolls the robot FORWARD", bottomRight),
                new Check("Intake motor (mainmotorintake)", "pulls a ball IN", intake),
                new Check("Left feeder  (leftfeeder)", "pulls a ball IN", leftFeeder),
                new Check("Right feeder (rightfeeder)", "pulls a ball IN", rightFeeder),
                new Check("Siphon (servosiphon)", "turns the carousel TOWARD the flywheel", siphon),
                new Check("Flywheel (turret)", "spins so a ball would launch OUT", flywheel));

        telemetry.addLine("Prop the robot up so the wheels are off the ground, then press START.");
        telemetry.addLine();
        addPortTelemetry(checks);
        telemetry.update();
        waitForStart();

        int selected = 0;
        int[] latched = new int[checks.size()];  // per device: +1 forward, -1 backward, 0 off
        while (opModeIsActive()) {
            if (gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed()) {
                selected = (selected + 1) % checks.size();
            }
            if (gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed()) {
                selected = (selected + checks.size() - 1) % checks.size();
            }
            if (gamepad1.xWasPressed() | gamepad2.xWasPressed()) {
                latched[selected] = latched[selected] == 1 ? 0 : 1;
            }
            if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) {
                latched[selected] = latched[selected] == -1 ? 0 : -1;
            }
            if (gamepad1.backWasPressed() | gamepad2.backWasPressed()) {
                Arrays.fill(latched, 0);
            }

            // A/B override the selected device while held; everything else follows its latch.
            boolean forward = gamepad1.a || gamepad2.a;   // either gamepad works
            boolean backward = gamepad1.b || gamepad2.b;
            int held = forward ? 1 : backward ? -1 : 0;

            telemetry.addLine("D-pad: pick.  Hold A/B: fwd/back.  X/Y: latch fwd/back.  Back: stop all.");
            for (int i = 0; i < checks.size(); i++) {
                int direction = (i == selected && held != 0) ? held : latched[i];
                checks.get(i).device.setPower(direction * TEST_POWER);
                String state = direction > 0 ? "[FWD]" : direction < 0 ? "[BACK]" : "[ off ]";
                telemetry.addLine((i == selected ? "> " : "   ") + state + " " + checks.get(i).label);
            }
            telemetry.addLine();
            telemetry.addData("Forward should", checks.get(selected).expected);
            telemetry.addLine();
            addPortTelemetry(checks);
            telemetry.update();
        }

        for (Check c : checks) {
            c.device.setPower(0.0);
        }
    }

    /** Lists the port each device is configured on, to compare with the list at the top of this file. */
    private void addPortTelemetry(List<Check> checks) {
        for (Check c : checks) {
            telemetry.addData(c.label, "port %d", c.port);
        }
    }
}
