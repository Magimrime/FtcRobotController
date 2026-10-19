package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Flywheel speed test. Spins the turret (flywheel) at a chosen RPM so you can find the speed that
 * scores, then feeds balls into it with the siphon. No driving. The flywheel starts spinning as
 * soon as you press START.
 *
 * Either gamepad works:
 *   Y (top button)          faster: target +100 RPM (or +5% power)
 *   A (bottom button)       weaker: target -100 RPM (or -5% power)
 *   Right stick up / down   speed up / slow down while held (the further, the faster it changes)
 *   D-pad up / down         target +100 / -100 RPM
 *   D-pad right / left      target +25 / -25 RPM
 *   L1 (left bumper)        flywheel off / on
 *   R2 (right trigger)   hold to run the siphon, which feeds balls into the flywheel
 *   L2 (left trigger)    press once: intake motor + both feeders ON and they stay on; press again: off
 *   X                    same as L2
 *   B                    switch between RPM mode (uses the encoder) and plain power mode
 *
 * The screen shows the target and the measured RPM, and says READY when they match. If the
 * measured RPM stays at 0 while the wheel is clearly spinning, the encoder cable is not plugged
 * in: press B and use power mode instead.
 *
 * RPM is worked out from FLYWHEEL_TICKS_PER_REV below, not from the motor type picked in the
 * Robot Configuration, so the numbers stay right even if the configuration says "Core Hex".
 *
 * Robot Configuration names:
 *   turret            flywheel motor, Expansion Hub motor 1
 *   servosiphon       siphon servo, Expansion Hub servo 1
 *   mainmotorintake   intake motor, Expansion Hub motor 0
 *   leftfeeder        intake servo, Control Hub servo 0
 *   rightfeeder       intake servo, Expansion Hub servo 0
 */
@TeleOp(name = "RPM Test", group = "Test")
public class RPMTest extends LinearOpMode {

    // The flywheel motor. The goBILDA StarterBot launcher is the 1:1 Yellow Jacket: 28 encoder
    // ticks per turn and about 6000 RPM free speed. Change both if a different motor is fitted.
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double FLYWHEEL_MAX_RPM = 6000.0;

    // Speed settings. RPM mode:
    private static final double START_RPM = 2700.0;     // roughly goBILDA's launcher target
    private static final double BIG_STEP_RPM = 100.0;   // d-pad up / down
    private static final double SMALL_STEP_RPM = 25.0;  // d-pad right / left
    private static final double READY_RPM = 75.0;       // READY when measured is within this of target
    private static final double STICK_RATE_RPM = 600.0; // right stick fully up: RPM added per second
    // Power mode:
    private static final double START_POWER = 0.70;
    private static final double BIG_STEP_POWER = 0.05;
    private static final double SMALL_STEP_POWER = 0.01;
    private static final double STICK_RATE_POWER = 0.30; // right stick fully up: power added per second
    private static final double STICK_DEADBAND = 0.1;    // ignore the stick inside this

    // Speed control inside the hub. F is 32767 / max ticks per second (how much motor power each
    // tick per second needs); P pulls the speed back up after each shot. goBILDA uses about these.
    private static final double PIDF_P = 40.0;
    private static final double PIDF_I = 0.0;
    private static final double PIDF_D = 0.0;
    private static final double PIDF_F = 32767.0 / (FLYWHEEL_MAX_RPM * FLYWHEEL_TICKS_PER_REV / 60.0);

    // Same as RobotTeleOp.
    private static final double INTAKE_POWER = 1.0;
    private static final double SIPHON_POWER = 1.0;
    private static final double SIPHON_TRIGGER = 0.1;  // right trigger past this = siphon on
    private static final double INTAKE_TRIGGER = 0.1;  // left trigger past this counts as a press
    private static final DcMotorSimple.Direction FLYWHEEL = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction INTAKE = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction LEFT_FEEDER = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction RIGHT_FEEDER = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction SIPHON = DcMotorSimple.Direction.REVERSE;

    @Override
    public void runOpMode() {
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        CRServo siphon = hardwareMap.get(CRServo.class, "servosiphon");
        DcMotor intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");

        flywheel.setDirection(FLYWHEEL);
        intake.setDirection(INTAKE);
        leftFeeder.setDirection(LEFT_FEEDER);
        rightFeeder.setDirection(RIGHT_FEEDER);
        siphon.setDirection(SIPHON);

        // Let the flywheel coast to a stop instead of braking it.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(PIDF_P, PIDF_I, PIDF_D, PIDF_F);

        String configType = flywheel.getMotorType().getName();
        double configTicks = flywheel.getMotorType().getTicksPerRev();

        telemetry.addLine("RPM Test. The flywheel spins up when you press START.");
        telemetry.addData("Config says turret is", "%s, %.0f ticks/rev (this file uses %.0f)",
                configType, configTicks, FLYWHEEL_TICKS_PER_REV);
        telemetry.update();
        waitForStart();

        boolean rpmMode = true;
        boolean flywheelOn = true;
        boolean intakeOn = false;
        boolean leftTriggerWasDown = false;  // triggers have no WasPressed(), so remember last loop
        double targetRpm = START_RPM;
        double targetPower = START_POWER;
        ElapsedTime loopTimer = new ElapsedTime();

        while (opModeIsActive()) {
            double seconds = loopTimer.seconds();  // time since the last loop, for the stick rate
            loopTimer.reset();

            // Buttons. The single | is on purpose: both gamepads get their press cleared.
            if (gamepad1.leftBumperWasPressed() | gamepad2.leftBumperWasPressed()) {
                flywheelOn = !flywheelOn;
            }
            // L2 or X: each press flips the intake, and it stays that way until the next press.
            boolean leftTriggerDown = gamepad1.left_trigger > INTAKE_TRIGGER || gamepad2.left_trigger > INTAKE_TRIGGER;
            boolean leftTriggerPressed = leftTriggerDown && !leftTriggerWasDown;
            leftTriggerWasDown = leftTriggerDown;
            if (leftTriggerPressed | gamepad1.xWasPressed() | gamepad2.xWasPressed()) {
                intakeOn = !intakeOn;
            }
            if (gamepad1.bWasPressed() | gamepad2.bWasPressed()) {
                rpmMode = !rpmMode;
                flywheel.setMode(rpmMode ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            double step = 0.0;  // big steps: Y (top button) or d-pad up = faster, A (bottom) or d-pad down = weaker
            if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) step += 1.0;
            if (gamepad1.aWasPressed() | gamepad2.aWasPressed()) step -= 1.0;
            if (gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed()) step += 1.0;
            if (gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed()) step -= 1.0;
            double smallStep = 0.0;
            if (gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed()) smallStep += 1.0;
            if (gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed()) smallStep -= 1.0;
            // Right stick: up is negative on a gamepad, so flip it. Holding it ramps the target.
            double stickUp = -stick(gamepad1.right_stick_y, gamepad2.right_stick_y);
            if (Math.abs(stickUp) < STICK_DEADBAND) stickUp = 0.0;
            if (rpmMode) {
                targetRpm += step * BIG_STEP_RPM + smallStep * SMALL_STEP_RPM + stickUp * STICK_RATE_RPM * seconds;
                targetRpm = Math.max(0.0, Math.min(FLYWHEEL_MAX_RPM, targetRpm));
            } else {
                targetPower += step * BIG_STEP_POWER + smallStep * SMALL_STEP_POWER + stickUp * STICK_RATE_POWER * seconds;
                targetPower = Math.max(0.0, Math.min(1.0, targetPower));
            }

            // Flywheel.
            if (!flywheelOn) {
                flywheel.setPower(0.0);
            } else if (rpmMode) {
                flywheel.setVelocity(targetRpm * FLYWHEEL_TICKS_PER_REV / 60.0);
            } else {
                flywheel.setPower(targetPower);
            }
            double measuredRpm = flywheel.getVelocity() * 60.0 / FLYWHEEL_TICKS_PER_REV;

            // Siphon feeds a ball while R2 is held. Intake + feeders run while toggled on.
            boolean siphonOn = gamepad1.right_trigger > SIPHON_TRIGGER || gamepad2.right_trigger > SIPHON_TRIGGER;
            siphon.setPower(siphonOn ? SIPHON_POWER : 0.0);
            intake.setPower(intakeOn ? INTAKE_POWER : 0.0);
            leftFeeder.setPower(intakeOn ? INTAKE_POWER : 0.0);
            rightFeeder.setPower(intakeOn ? INTAKE_POWER : 0.0);

            telemetry.addData("Mode", rpmMode ? "RPM (encoder)   [B: power mode]" : "POWER   [B: RPM mode]");
            telemetry.addData("Flywheel", flywheelOn ? "ON   [L1: off]" : "OFF   [L1: on]");
            if (rpmMode) {
                String state = !flywheelOn ? "" : Math.abs(measuredRpm - targetRpm) <= READY_RPM ? "   READY" : "   spinning up";
                telemetry.addData("Target", "%.0f RPM   [Y faster, A weaker (100); right stick; d-pad right/left 25]", targetRpm);
                telemetry.addData("Measured", "%.0f RPM%s", measuredRpm, state);
            } else {
                telemetry.addData("Power", "%.0f%%   [Y faster, A weaker (5); right stick; d-pad right/left 1]", targetPower * 100.0);
                telemetry.addData("Measured", "%.0f RPM", measuredRpm);
            }
            telemetry.addData("Siphon", siphonOn ? "ON (R2 held)" : "off   [hold R2 to feed]");
            telemetry.addData("Intake", intakeOn ? "ON   [L2 or X: off]" : "off   [L2 or X: on]");
            telemetry.addData("Config says turret is", "%s, %.0f ticks/rev", configType, configTicks);
            telemetry.update();
        }

        flywheel.setPower(0.0);
        siphon.setPower(0.0);
        intake.setPower(0.0);
        leftFeeder.setPower(0.0);
        rightFeeder.setPower(0.0);
    }

    /** One stick axis from both gamepads: whichever is pushed further wins. */
    private static double stick(float pad1, float pad2) {
        return Math.abs(pad1) >= Math.abs(pad2) ? pad1 : pad2;
    }
}
