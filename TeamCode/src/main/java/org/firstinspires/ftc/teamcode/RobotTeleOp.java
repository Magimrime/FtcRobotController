package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

/*
 * Driver-controlled mode for the whole robot, all in this one file: mecanum drive, intake and
 * shooter. Every motor and servo is looked up by name in initHardware() below.
 *
 * HARDWARE (name in the Robot Configuration -> type -> where it plugs in):
 *   topleft           Motor                       Control Hub motor port 0     front left wheel
 *   topright          Motor                       Control Hub motor port 1     front right wheel
 *   bottomleft        Motor                       Control Hub motor port 2     back left wheel
 *   bottomright       Motor                       Control Hub motor port 3     back right wheel
 *   leftfeeder        Continuous Rotation Servo   Control Hub servo port 0     left intake feeder
 *   mainmotorintake   Motor                       Expansion Hub motor port 0   intake motor
 *   turret            Motor (set the real model)  Expansion Hub motor port 1   flywheel
 *   rightfeeder       Continuous Rotation Servo   Expansion Hub servo port 0   right intake feeder
 *   servosiphon       Continuous Rotation Servo   Expansion Hub servo port 1   turns the carousel
 *
 * Names are case sensitive. If one cannot be found, INIT stops with a red message that names it
 * and lists the devices and hubs the robot does have. If that list says "Hubs: Control Hub" and
 * nothing else, the Expansion Hub had no power or no RS-485 link to the Control Hub when the
 * robot started (the Driver Station also says "Expansion Hub 2 was missing at startup"). Fix the
 * cable or the power, then Restart Robot on the Driver Station.
 *
 * WHEELS, viewed from above. The intake is the front. The rollers on top of each wheel must
 * point toward the center of the robot, so the four of them form an X (gm0.org, Mecanum Drive).
 *
 *                  FRONT (intake)
 *          topleft                topright
 *           port 0                port 1
 *             \\                     //
 *
 *             //                     \\
 *           port 2                port 3
 *        bottomleft              bottomright
 *
 * The drive math and the motor directions are gm0.org's Mecanum Drive tutorial: the RIGHT side
 * motors are reversed. If the robot drives BACKWARD when the stick is pushed forward, reverse
 * the left side instead (see initHardware). If only one wheel is wrong, hold Back and press a
 * d-pad direction to run just that wheel (wheel test), then flip the one that rolls backward.
 *
 * CONTROLS, from EITHER gamepad (whichever stick is pushed further drives):
 *   Left stick       forward / back / strafe
 *   Right stick X    turn
 *   Right bumper     hold for half speed
 *   Back + d-pad     wheel test: up = front left, right = front right, left = back left,
 *                    down = back right. Only that wheel runs, slowly. It must roll FORWARD.
 *   Left trigger     hold to run the intake backward and clear a jam
 *   Right trigger    hold to pause the intake
 *   Y                flywheel off / on
 *   D-pad up / down  flywheel power up / down by 5% (also during INIT)
 *   A                siphon (carousel) on / off
 *   B                hold to run the carousel backward
 *
 * The intake (motor and both feeders) and the flywheel start the moment the match starts and run
 * all match. The flywheel runs at full power by default, the fastest the motor can turn. It is
 * driven by plain power, not a speed target, so it goes flat out even if the motor model in the
 * Robot Configuration is wrong; only the rpm readout on the screen depends on that model.
 *
 * Nothing moves during INIT.
 */
@TeleOp(name = "Robot TeleOp", group = "Robot")
public class RobotTeleOp extends LinearOpMode {

    // ---------- Hardware names. These must match the Robot Configuration exactly. ----------
    private static final String FRONT_LEFT_NAME = "topleft";
    private static final String FRONT_RIGHT_NAME = "topright";
    private static final String BACK_LEFT_NAME = "bottomleft";
    private static final String BACK_RIGHT_NAME = "bottomright";
    private static final String INTAKE_MOTOR_NAME = "mainmotorintake";
    private static final String LEFT_FEEDER_NAME = "leftfeeder";
    private static final String RIGHT_FEEDER_NAME = "rightfeeder";
    private static final String FLYWHEEL_NAME = "turret";
    private static final String SIPHON_NAME = "servosiphon";

    // ---------- Tuning. ----------
    private static final double STRAFE_CORRECTION = 1.1;     // counteracts imperfect strafing (gm0)
    private static final double SLOW_MODE_POWER = 0.5;       // wheel power limit while RB is held
    private static final double WHEEL_TEST_POWER = 0.3;
    private static final double INTAKE_POWER = 1.0;          // positive pulls balls in
    private static final double INTAKE_REVERSE_POWER = 0.7;
    private static final double FLYWHEEL_START_POWER = 1.0;  // 1.0 = full power, the hardest shot
    private static final double FLYWHEEL_POWER_STEP = 0.05;
    private static final double SIPHON_POWER = 1.0;          // positive moves balls to the flywheel

    // ---------- Hardware. ----------
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeMotor;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private DcMotorEx flywheel;
    private CRServo siphon;
    private double flywheelTicksPerRev;
    private String flywheelMotorModel;

    // ---------- State. ----------
    private boolean flywheelOn = false;
    private double flywheelPower = FLYWHEEL_START_POWER;
    private boolean siphonOn = false;

    // The last powers sent, for the screen.
    private double frontLeftSent;
    private double frontRightSent;
    private double backLeftSent;
    private double backRightSent;
    private double intakeSent;
    private double siphonSent;

    @Override
    public void runOpMode() {
        initHardware();

        while (opModeInInit()) {
            readFlywheelPowerButtons();  // the flywheel power can be set before the match
            telemetry.addLine("Ready. The intake and the flywheel start when the match starts.");
            telemetry.addData("Flywheel power", "%.0f%%   (d-pad up / down to change)", flywheelPower * 100);
            addPortTelemetry();
            telemetry.update();
        }

        // Button presses are remembered until read, so a Y or A pressed during INIT would act the
        // moment the match starts. Read and ignore them.
        gamepad1.yWasPressed();
        gamepad2.yWasPressed();
        gamepad1.aWasPressed();
        gamepad2.aWasPressed();

        // The match has started: the flywheel comes on here and the intake in runIntake().
        flywheelOn = true;

        while (opModeIsActive()) {
            drive();
            runIntake();
            runShooter();
            addTelemetry();
            telemetry.update();
        }

        stopEverything();
    }

    // =========================================================================================
    // HARDWARE SETUP
    // =========================================================================================

    /** Finds every device by name and sets its direction and mode. */
    private void initHardware() {
        frontLeft = find(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = find(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = find(DcMotor.class, BACK_LEFT_NAME);
        backRight = find(DcMotor.class, BACK_RIGHT_NAME);
        intakeMotor = find(DcMotor.class, INTAKE_MOTOR_NAME);
        leftFeeder = find(CRServo.class, LEFT_FEEDER_NAME);
        rightFeeder = find(CRServo.class, RIGHT_FEEDER_NAME);
        flywheel = find(DcMotorEx.class, FLYWHEEL_NAME);
        siphon = find(CRServo.class, SIPHON_NAME);

        // ----- Drive -----
        // Reverse the right side motors (gm0). Most FTC motors spin counterclockwise when given
        // positive power, and the two sides face opposite ways, so one side must be reversed.
        // If the robot drives BACKWARD when the stick is pushed forward, reverse the LEFT side
        // instead: REVERSE on the two left motors and FORWARD on the two right ones.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  // stop quickly when released
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ----- Intake -----
        // Positive power must pull balls IN on all three. The two feeders face each other across
        // the intake, so one of them is reversed. Flip FORWARD/REVERSE on anything that pushes
        // balls out instead.
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ----- Shooter -----
        // Positive flywheel power must launch balls OUT, and positive siphon power must move balls
        // toward the flywheel. Flip FORWARD/REVERSE on anything that runs the wrong way.
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        siphon.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // coast down, easier on gears
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);           // plain power, see header

        // Only the rpm readout uses the motor model from the Robot Configuration.
        MotorConfigurationType motorType = flywheel.getMotorType();
        flywheelTicksPerRev = motorType.getTicksPerRev();
        flywheelMotorModel = motorType.getName();
    }

    /**
     * Looks a device up by name. If it cannot be found, the red error that stops INIT names the
     * missing device and lists the devices and hubs the robot does have, so a typo in the Robot
     * Configuration or a hub the Control Hub cannot reach shows up right away instead of a bare
     * "Unable to find a hardware device".
     */
    private <T extends HardwareDevice> T find(Class<T> type, String name) {
        try {
            return hardwareMap.get(type, name);
        } catch (IllegalArgumentException notFound) {
            String kind = CRServo.class.isAssignableFrom(type) ? "CR servo"
                    : DcMotor.class.isAssignableFrom(type) ? "motor" : type.getSimpleName();
            throw new IllegalArgumentException("No " + kind + " named \"" + name
                    + "\" (names are case sensitive). " + kind + "s the robot has: "
                    + namesOf(hardwareMap.getAll(type))
                    + ". Hubs the Control Hub can talk to: " + namesOf(hardwareMap.getAll(LynxModule.class))
                    + ". A hub missing from that list has no power or no RS-485 link to the Control"
                    + " Hub: check its cable and LED, then Restart Robot. Otherwise fix the name in"
                    + " the Robot Configuration or activate the right configuration.",
                    notFound);
        }
    }

    /** The configuration names of some devices, comma separated, or "none". */
    private String namesOf(Iterable<? extends HardwareDevice> devices) {
        StringBuilder names = new StringBuilder();
        for (HardwareDevice device : devices) {
            for (String deviceName : hardwareMap.getNamesOf(device)) {
                names.append(names.length() == 0 ? "" : ", ").append(deviceName);
            }
        }
        return names.length() == 0 ? "none" : names.toString();
    }

    // =========================================================================================
    // DRIVING
    // =========================================================================================

    /**
     * Mecanum drive, straight from gm0.org's Mecanum Drive tutorial. Each wheel's power is the
     * sum of what forward, strafe and turn ask of it. Whichever gamepad is pushed further drives.
     */
    private void drive() {
        if (gamepad1.back || gamepad2.back) {
            runWheelTest();
            return;
        }

        double y = -stick(gamepad1.left_stick_y, gamepad2.left_stick_y);  // stick Y is reversed
        double x = stick(gamepad1.left_stick_x, gamepad2.left_stick_x) * STRAFE_CORRECTION;
        double rx = stick(gamepad1.right_stick_x, gamepad2.right_stick_x);

        // Denominator is the largest motor power (absolute value) or 1. This ensures all the
        // powers maintain the same ratio, but only if at least one is out of the range [-1, 1].
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Right bumper: slow mode, for lining up.
        double scale = (gamepad1.right_bumper || gamepad2.right_bumper) ? SLOW_MODE_POWER : 1.0;
        setWheelPowers(frontLeftPower * scale, frontRightPower * scale,
                backLeftPower * scale, backRightPower * scale);
    }

    /**
     * Wheel test, for finding a wheel that is wired or reversed wrong. While Back is held, a d-pad
     * direction runs ONE wheel slowly forward. Put the robot on blocks. Only that wheel should
     * turn, the way that would roll the robot FORWARD. A different wheel turning means the names
     * are on the wrong ports in the Robot Configuration. The right wheel turning backward means
     * its direction in initHardware() must be flipped.
     */
    private void runWheelTest() {
        boolean fl = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean fr = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean bl = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean br = gamepad1.dpad_down || gamepad2.dpad_down;
        setWheelPowers(fl ? WHEEL_TEST_POWER : 0.0, fr ? WHEEL_TEST_POWER : 0.0,
                bl ? WHEEL_TEST_POWER : 0.0, br ? WHEEL_TEST_POWER : 0.0);

        telemetry.addData("WHEEL TEST", fl ? "FRONT LEFT (topleft)" : fr ? "FRONT RIGHT (topright)"
                : bl ? "BACK LEFT (bottomleft)" : br ? "BACK RIGHT (bottomright)"
                : "d-pad: up=front left  right=front right  left=back left  down=back right");
        telemetry.addLine("Only that wheel should turn, the way that rolls the robot FORWARD.");
    }

    /** Sets each wheel's power, -1 to 1. Positive rolls that wheel forward. */
    private void setWheelPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
        frontLeftSent = fl;
        frontRightSent = fr;
        backLeftSent = bl;
        backRightSent = br;
    }

    // =========================================================================================
    // INTAKE
    // =========================================================================================

    /**
     * The intake motor and both feeders run the whole match, pulling balls in. Hold the left
     * trigger to run them backward and clear a jam, or the right trigger to pause them.
     */
    private void runIntake() {
        double power = INTAKE_POWER;
        if (gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) {
            power = -INTAKE_REVERSE_POWER;
        } else if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
            power = 0.0;
        }
        intakeMotor.setPower(power);
        leftFeeder.setPower(power);
        rightFeeder.setPower(power);
        intakeSent = power;
    }

    // =========================================================================================
    // SHOOTER
    // =========================================================================================

    /**
     * The flywheel runs from the start of the match. Y turns it off and on, d-pad up / down
     * changes its power. A turns the siphon on and off; holding B runs the carousel backward.
     */
    private void runShooter() {
        // Read BOTH pads' presses every loop (single |, not ||) so neither pad's press is left
        // waiting to fire later.
        if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) {
            flywheelOn = !flywheelOn;
        }
        readFlywheelPowerButtons();
        flywheel.setPower(flywheelOn ? flywheelPower : 0.0);

        if (gamepad1.aWasPressed() | gamepad2.aWasPressed()) {
            siphonOn = !siphonOn;
        }
        double power = (gamepad1.b || gamepad2.b) ? -SIPHON_POWER : siphonOn ? SIPHON_POWER : 0.0;
        siphon.setPower(power);
        siphonSent = power;
    }

    /** D-pad up / down changes the flywheel power one step per press (not during the wheel test). */
    private void readFlywheelPowerButtons() {
        boolean up = gamepad1.dpadUpWasPressed() | gamepad2.dpadUpWasPressed();
        boolean down = gamepad1.dpadDownWasPressed() | gamepad2.dpadDownWasPressed();
        if (gamepad1.back || gamepad2.back) {
            return;  // the d-pad belongs to the wheel test while Back is held
        }
        if (up) {
            flywheelPower = Math.min(flywheelPower + FLYWHEEL_POWER_STEP, 1.0);
        }
        if (down) {
            flywheelPower = Math.max(flywheelPower - FLYWHEEL_POWER_STEP, 0.0);
        }
    }

    /** The flywheel's measured speed in rpm, from its encoder, assuming the configured motor model. */
    private double flywheelRpm() {
        return flywheelTicksPerRev > 0.0 ? flywheel.getVelocity() * 60.0 / flywheelTicksPerRev : 0.0;
    }

    /** STOP was pressed: leave everything stopped. */
    private void stopEverything() {
        setWheelPowers(0.0, 0.0, 0.0, 0.0);
        intakeMotor.setPower(0.0);
        leftFeeder.setPower(0.0);
        rightFeeder.setPower(0.0);
        flywheel.setPower(0.0);
        siphon.setPower(0.0);
    }

    // =========================================================================================
    // SCREEN
    // =========================================================================================

    private void addTelemetry() {
        telemetry.addData("Flywheel", "%s   power %.0f%%   %.0f rpm", flywheelOn ? "ON" : "OFF",
                flywheelPower * 100, flywheelRpm());
        telemetry.addData("Intake", intakeSent > 0.0 ? "In" : intakeSent < 0.0 ? "Reverse" : "Paused");
        telemetry.addData("Siphon", siphonSent > 0.0 ? "Feeding" : siphonSent < 0.0 ? "Reverse" : "Stopped");
        telemetry.addData("Wheels", "FL %5.2f  FR %5.2f  BL %5.2f  BR %5.2f",
                frontLeftSent, frontRightSent, backLeftSent, backRightSent);
        telemetry.addLine("Y: flywheel   D-pad: flywheel power   A / B: siphon   LT: intake back   RT: pause");
        telemetry.addLine("RB: slow   Back + d-pad: wheel test");
    }

    /** INIT screen: the port each name was found on, to compare with the table at the top. */
    private void addPortTelemetry() {
        telemetry.addData("Drive ports", "topleft %d  topright %d  bottomleft %d  bottomright %d",
                frontLeft.getPortNumber(), frontRight.getPortNumber(),
                backLeft.getPortNumber(), backRight.getPortNumber());
        telemetry.addData("Intake ports", "mainmotorintake %d  leftfeeder %d  rightfeeder %d",
                intakeMotor.getPortNumber(), leftFeeder.getPortNumber(), rightFeeder.getPortNumber());
        telemetry.addData("Shooter ports", "turret %d  servosiphon %d",
                flywheel.getPortNumber(), siphon.getPortNumber());
        telemetry.addData("Flywheel motor model", "%s (only the rpm readout depends on this)",
                flywheelMotorModel);
    }

    // =========================================================================================
    // GAMEPAD HELPER
    // =========================================================================================

    /**
     * One stick axis from both gamepads: whichever is pushed further wins. This is what lets
     * either pad drive, including a pad the Driver Station registered as user 2 (Start+B).
     */
    private static double stick(float pad1Value, float pad2Value) {
        return Math.abs(pad1Value) >= Math.abs(pad2Value) ? pad1Value : pad2Value;
    }
}
