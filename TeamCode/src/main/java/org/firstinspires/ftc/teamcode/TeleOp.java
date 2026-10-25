package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * goBILDA mecanum StarterBot (BIOBUZZ). Left stick drives and strafes, right stick turns.
 * Either gamepad works.
 *
 * The two side feeder servos, the intake motor and the flywheel start spinning at START and run
 * for the whole match. The siphon only turns while R2 (the right trigger) is held, so balls go
 * into the flywheel when the driver asks, not all the time.
 *
 * Clearing jams, both spring back to normal as soon as you let go:
 *   hold L2 (left trigger)    intake motor and both feeders run backward
 *   hold R1 (right bumper)    siphon runs backward (wins over R2 if both are held)
 *
 * Flywheel speed: starts at 2800 RPM. The four buttons on the right side of the gamepad set it:
 *   top button (triangle / Y)     +100 RPM
 *   bottom button (cross / A)     -100 RPM
 * It stays between 2600 and 3500 and is always a round multiple of 100. The hub holds the speed
 * using the motor encoder, so it stays the same as the battery runs down.
 *
 * To test one wheel at a time, use the "Robot Hardware Test" utility on the Driver Station.
 *
 * Robot Configuration names:
 *   topleft, topright, bottomleft, bottomright   wheel motors, Control Hub motor ports 0-3
 *   leftfeeder                                   intake servo, Control Hub servo 0
 *   mainmotorintake                              intake motor, Expansion Hub motor 0
 *   turret                                       flywheel motor, Expansion Hub motor 1
 *   rightfeeder                                  intake servo, Expansion Hub servo 0
 *   servosiphon                                  siphon servo, Expansion Hub servo 1
 */
// The annotation is written out in full because this class is itself called TeleOp.
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "Robot")
public class TeleOp extends LinearOpMode {

    // Wheel directions, from the wheel test on 2026-10-05: with everything FORWARD only the front
    // right wheel rolled forward, so the other three are reversed here. Redo the test after any
    // rewiring: a wheel that rolls backward under its button needs the opposite setting.
    private static final DcMotorSimple.Direction TOP_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction TOP_RIGHT = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction BOTTOM_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction BOTTOM_RIGHT = DcMotorSimple.Direction.REVERSE;

    // How hard things spin, 0 to 1.
    private static final double INTAKE_POWER = 1.0;
    private static final double SIPHON_POWER = 1.0;
    private static final double SIPHON_TRIGGER = 0.1;  // right trigger past this = siphon on
    private static final double REVERSE_TRIGGER = 0.1; // left trigger past this = intake backward

    // Flywheel speed, in RPM. Top / bottom face button moves it one step; it never leaves MIN..MAX
    // and is always rounded to a whole step.
    private static final double FLYWHEEL_START_RPM = 2800.0;
    private static final double FLYWHEEL_MIN_RPM = 2600.0;
    private static final double FLYWHEEL_MAX_RPM = 3500.0;
    private static final double FLYWHEEL_STEP_RPM = 100.0;

    // The flywheel motor, same numbers as RPMTest: goBILDA 1:1 Yellow Jacket, 28 encoder ticks
    // per turn, about 6000 RPM free speed. The PIDF lets the hub hold a speed on its own.
    private static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    private static final double FLYWHEEL_FREE_RPM = 6000.0;
    private static final double FLYWHEEL_P = 40.0;
    private static final double FLYWHEEL_I = 0.0;
    private static final double FLYWHEEL_D = 0.0;
    private static final double FLYWHEEL_F = 32767.0 / (FLYWHEEL_FREE_RPM * FLYWHEEL_TICKS_PER_REV / 60.0);

    @Override
    public void runOpMode() {
        DcMotor topLeft = hardwareMap.get(DcMotor.class, "topleft");
        DcMotor topRight = hardwareMap.get(DcMotor.class, "topright");
        DcMotor bottomLeft = hardwareMap.get(DcMotor.class, "bottomleft");
        DcMotor bottomRight = hardwareMap.get(DcMotor.class, "bottomright");
        DcMotor intake = hardwareMap.get(DcMotor.class, "mainmotorintake");
        CRServo leftFeeder = hardwareMap.get(CRServo.class, "leftfeeder");
        CRServo rightFeeder = hardwareMap.get(CRServo.class, "rightfeeder");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "turret");
        CRServo siphon = hardwareMap.get(CRServo.class, "servosiphon");

        topLeft.setDirection(TOP_LEFT);
        topRight.setDirection(TOP_RIGHT);
        bottomLeft.setDirection(BOTTOM_LEFT);
        bottomRight.setDirection(BOTTOM_RIGHT);
        for (DcMotor wheel : new DcMotor[] {topLeft, topRight, bottomLeft, bottomRight}) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // stop quickly when released
        }
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);  // the two feeders face each other
        siphon.setDirection(DcMotorSimple.Direction.REVERSE);       // goBILDA's windmill direction

        // Flywheel: let it coast when stopped, and let the hub hold the speed from the encoder.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        telemetry.addLine("Ready. Feeders, intake and flywheel start when the match starts.");
        telemetry.addLine("Hold R2 (right trigger) to run the siphon. Triangle / cross changes flywheel speed.");
        telemetry.addLine("Jams: hold L2 to reverse the intake, hold R1 to reverse the siphon.");
        telemetry.update();
        waitForStart();

        // Everything that just spins for the whole match. The siphon is on R2, below.
        intake.setPower(INTAKE_POWER);
        leftFeeder.setPower(INTAKE_POWER);
        rightFeeder.setPower(INTAKE_POWER);
        boolean intakeReversed = false;
        double flywheelRpm = FLYWHEEL_START_RPM;
        flywheel.setVelocity(flywheelRpm * FLYWHEEL_TICKS_PER_REV / 60.0);

        while (opModeIsActive()) {
            // Flywheel speed: top face button (triangle / Y) up one step, bottom (cross / A) down one
            // step, either gamepad. The single | is on purpose so both gamepads get their press
            // cleared. Rounded to a whole step and kept inside MIN..MAX, then sent to the hub only
            // when it actually changes.
            double wantedRpm = flywheelRpm;
            if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) wantedRpm += FLYWHEEL_STEP_RPM;
            if (gamepad1.aWasPressed() | gamepad2.aWasPressed()) wantedRpm -= FLYWHEEL_STEP_RPM;
            wantedRpm = Math.round(wantedRpm / FLYWHEEL_STEP_RPM) * FLYWHEEL_STEP_RPM;
            wantedRpm = Math.max(FLYWHEEL_MIN_RPM, Math.min(FLYWHEEL_MAX_RPM, wantedRpm));
            if (wantedRpm != flywheelRpm) {
                flywheelRpm = wantedRpm;
                flywheel.setVelocity(flywheelRpm * FLYWHEEL_TICKS_PER_REV / 60.0);
            }
            double measuredRpm = flywheel.getVelocity() * 60.0 / FLYWHEEL_TICKS_PER_REV;

            // Siphon: forward while R2 is held, backward while R1 is held (R1 wins), otherwise off.
            // Either gamepad. Sent every loop, so letting go returns it to normal at once.
            boolean siphonOn = gamepad1.right_trigger > SIPHON_TRIGGER || gamepad2.right_trigger > SIPHON_TRIGGER;
            boolean siphonReversed = gamepad1.right_bumper || gamepad2.right_bumper;
            siphon.setPower(siphonReversed ? -SIPHON_POWER : siphonOn ? SIPHON_POWER : 0.0);

            // Intake: runs backward while L2 is held on either gamepad, forward otherwise. The
            // powers are only re-sent when the direction actually changes.
            boolean reverseWanted = gamepad1.left_trigger > REVERSE_TRIGGER || gamepad2.left_trigger > REVERSE_TRIGGER;
            if (reverseWanted != intakeReversed) {
                intakeReversed = reverseWanted;
                double intakePower = intakeReversed ? -INTAKE_POWER : INTAKE_POWER;
                intake.setPower(intakePower);
                leftFeeder.setPower(intakePower);
                rightFeeder.setPower(intakePower);
            }

            // Whichever gamepad is pushed further drives. Stick Y reads negative when pushed forward.
            double forward = -stick(gamepad1.left_stick_y, gamepad2.left_stick_y);
            double strafe = stick(gamepad1.left_stick_x, gamepad2.left_stick_x);
            double turn = stick(gamepad1.right_stick_x, gamepad2.right_stick_x);

            // Mecanum: each wheel adds up what forward, strafe and turn ask of it (goBILDA / gm0).
            double fl = forward + strafe + turn;
            double fr = forward - strafe - turn;
            double bl = forward - strafe + turn;
            double br = forward + strafe - turn;

            // Keep every wheel within -1..1 without changing the ratios between them.
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));
            if (max > 1.0) {
                fl /= max;
                fr /= max;
                bl /= max;
                br /= max;
            }

            topLeft.setPower(fl);
            topRight.setPower(fr);
            bottomLeft.setPower(bl);
            bottomRight.setPower(br);

            telemetry.addData("Sticks", "GP1 L(%+.2f,%+.2f) R(%+.2f)   GP2 L(%+.2f,%+.2f) R(%+.2f)",
                    gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x,
                    gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);
            telemetry.addData("Wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", fl, fr, bl, br);
            telemetry.addData("Siphon", siphonReversed ? "BACKWARD (R1 held)" : siphonOn ? "ON (R2 held)" : "off  (hold R2, R1 reverses)");
            telemetry.addData("Intake", intakeReversed ? "BACKWARD (L2 held)" : "forward  (hold L2 to reverse)");
            telemetry.addData("Flywheel", "%.0f RPM wanted, %.0f now   [triangle up, cross down, %.0f-%.0f]",
                    flywheelRpm, measuredRpm, FLYWHEEL_MIN_RPM, FLYWHEEL_MAX_RPM);
            telemetry.update();
        }
    }

    /** One stick axis from both gamepads: whichever is pushed further wins. */
    private static double stick(float pad1, float pad2) {
        return Math.abs(pad1) >= Math.abs(pad2) ? pad1 : pad2;
    }
}
