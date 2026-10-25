package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * Flywheel speed: starts at 3000 RPM. D-pad up adds 100, d-pad down takes 100 away, between
 * 3000 and 3500. The speed is always a round multiple of 100. The hub holds the speed using the
 * motor encoder, so it stays the same as the battery runs down.
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

    // How hard things spin, 0 to 1.
    private static final double INTAKE_POWER = 1.0;
    private static final double SIPHON_POWER = 1.0;
    private static final double SIPHON_TRIGGER = 0.1;  // right trigger past this = siphon on
    private static final double DEBUG_WHEEL_POWER = 0.3;  // one wheel at a time, button held

    // Flywheel speed, in RPM. D-pad up / down moves it one step; it never leaves MIN..MAX and is
    // always rounded to a whole step.
    private static final double FLYWHEEL_START_RPM = 3000.0;
    private static final double FLYWHEEL_MIN_RPM = 3000.0;
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
        telemetry.addLine("Hold R2 (right trigger) to run the siphon. D-pad up/down changes flywheel speed.");
        telemetry.update();
        waitForStart();

        // Everything that just spins for the whole match. The siphon is on R2, below.
        intake.setPower(INTAKE_POWER);
        leftFeeder.setPower(INTAKE_POWER);
        rightFeeder.setPower(INTAKE_POWER);
        double flywheelRpm = FLYWHEEL_START_RPM;
        flywheel.setVelocity(flywheelRpm * FLYWHEEL_TICKS_PER_REV / 60.0);

        while (opModeIsActive()) {
            // Flywheel speed: d-pad up / down on either gamepad moves it one step. The single | is
            // on purpose so both gamepads get their press cleared. Rounded to a whole step and kept
            // inside MIN..MAX, then sent to the hub only when it actually changes.
            double wantedRpm = flywheelRpm;
            if (gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed()) wantedRpm += FLYWHEEL_STEP_RPM;
            if (gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed()) wantedRpm -= FLYWHEEL_STEP_RPM;
            wantedRpm = Math.round(wantedRpm / FLYWHEEL_STEP_RPM) * FLYWHEEL_STEP_RPM;
            wantedRpm = Math.max(FLYWHEEL_MIN_RPM, Math.min(FLYWHEEL_MAX_RPM, wantedRpm));
            if (wantedRpm != flywheelRpm) {
                flywheelRpm = wantedRpm;
                flywheel.setVelocity(flywheelRpm * FLYWHEEL_TICKS_PER_REV / 60.0);
            }
            double measuredRpm = flywheel.getVelocity() * 60.0 / FLYWHEEL_TICKS_PER_REV;

            // Siphon: turns the carousel only while R2 is held on either gamepad.
            boolean siphonOn = gamepad1.right_trigger > SIPHON_TRIGGER || gamepad2.right_trigger > SIPHON_TRIGGER;
            siphon.setPower(siphonOn ? SIPHON_POWER : 0.0);

            double fl, fr, bl, br;

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
            telemetry.addData("Siphon", siphonOn ? "ON (R2 held)" : "off  (hold R2)");
            telemetry.addData("Flywheel", "%.0f RPM wanted, %.0f now   [d-pad up/down, %.0f-%.0f]",
                    flywheelRpm, measuredRpm, FLYWHEEL_MIN_RPM, FLYWHEEL_MAX_RPM);
            telemetry.addLine("Hold triangle=front left  circle=front right  square=back left  cross=back right");
            telemetry.update();
        }
    }

    /** One stick axis from both gamepads: whichever is pushed further wins. */
    private static double stick(float pad1, float pad2) {
        return Math.abs(pad1) >= Math.abs(pad2) ? pad1 : pad2;
    }
}
