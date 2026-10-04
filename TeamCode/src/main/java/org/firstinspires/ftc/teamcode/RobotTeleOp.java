package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * Driver-controlled mode for the whole robot, all in this one file: mecanum drive, intake and
 * shooter. Every motor and servo is looked up by name in initHardware() below, so the names to
 * type into the Robot Configuration on the Driver Station are all listed right here.
 *
 * HARDWARE (name in the Robot Configuration -> type -> where it plugs in):
 *   topleft           Motor                       Control Hub motor port 0     front left wheel
 *   topright          Motor                       Control Hub motor port 1     front right wheel
 *   bottomleft        Motor                       Control Hub motor port 2     back left wheel
 *   bottomright       Motor                       Control Hub motor port 3     back right wheel
 *   mainmotorintake   Motor                       Expansion Hub motor port 0   intake motor
 *   leftfeeder        Continuous Rotation Servo   Control Hub servo port 0     left intake feeder
 *   rightfeeder       Continuous Rotation Servo   Expansion Hub servo port 0   right intake feeder
 *   turret            Motor (set the real model)  Expansion Hub motor port 1   flywheel, encoder in
 *   servosiphon       Continuous Rotation Servo   Expansion Hub servo port 1   turns the carousel
 *
 * The names are case sensitive. If any one of them is not in the configuration, INIT stops with
 * a red "Unable to find a hardware device with name ..." message and nothing runs, driving
 * included. The INIT screen lists the port each name was found on, to compare with the table.
 *
 * SERVOS THAT WILL NOT SPIN CONTINUOUSLY: this code sends the three servos a steady full-speed
 * continuous-rotation signal. A servo that only moves to one spot and stops is in positional
 * mode inside the servo itself, which no code can change. goBILDA dual-mode servos and REV Smart
 * Robot Servos ship in positional mode and must be switched to continuous mode with their
 * programmer tool. A plain hobby servo cannot spin continuously at all. The Robot Configuration
 * entry must also be "Continuous Rotation Servo", not "Servo".
 *
 * Wheel layout viewed from above. The intake is the front, so driving forward goes intake-first.
 * The rollers on top of the wheels must form an "X" like this, otherwise strafing goes the wrong
 * way.
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
 * Every control works from EITHER gamepad: one driver can run the whole robot from one pad, or two
 * drivers can split the work however they like. Whichever pad's stick is pushed further drives.
 *
 * Driving (either gamepad). Plain robot-relative mecanum: no IMU, no field-centric, no assists.
 *   Left stick        move: forward, back, strafe left/right, or any diagonal, relative to the
 *                     robot. On its own it never turns the robot.
 *   Right stick X     turn left / right. On its own the robot spins in place. Pushed together
 *                     with the left stick, the robot moves and turns at the same time.
 *   Right bumper      hold for precision mode: slower, for lining up
 *   D-pad             nudge slowly forward / back / left / right (overrides the left stick)
 *   Back (Share) +    WHEEL TEST, for finding a wheel that is wired or reversed wrong: hold Back
 *     d-pad           and press d-pad up = front left, right = front right, left = back left,
 *                     down = back right. That ONE wheel runs slowly and must roll the robot
 *                     FORWARD. Put the robot on blocks first. The screen shows each wheel's
 *                     motor current and encoder speed, to compare a weak wheel with a good one.
 *
 * Mechanisms (either gamepad):
 *   Right trigger     hold to run the intake motor, pulling balls in
 *   Left trigger      hold to run the intake backwards, pushing balls out
 *   Y                 flywheel on / off
 *   A                 siphon on / off: press once to start the carousel, again to stop it
 *   B                 hold to turn the carousel backward, to clear a jam
 *   LB + d-pad        hold the left bumper and press up / down to change the flywheel speed
 *                     by 100 RPM (without the bumper held, the d-pad nudges)
 *
 * The two feeder servos at the intake run the whole match, pulling balls in. They only reverse
 * while the left trigger is pushing balls out.
 *
 * The flywheel is given a speed in RPM, not a power. The hub's built-in speed controller reads
 * the encoder and adjusts the power to hold that speed, so shots stay the same as the battery
 * drains. It starts at the motor's full speed, the hardest shot the motor can give; LB + d-pad
 * lowers it for shorter shots, and FLYWHEEL_DEFAULT_SPEED below sets where it starts. This only
 * works if the "turret" motor in the Robot Configuration is set to the real motor model, not
 * "Unspecified Motor": the RPM figures come from the model's encoder ticks per turn and top
 * speed. The INIT screen shows the model the robot is using and warns if it is not set.
 *
 * Nothing moves during INIT, but the flywheel speed can be set then.
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

    // ---------- Driving. ----------
    // Sticks closer to center than this count as centered, so worn sticks don't creep the robot.
    private static final double STICK_DEADBAND = 0.05;
    // Stick response: 1 = linear, 2 = squared, 3 = cubed. Higher gives finer control at low
    // speed while still reaching full speed at full stick.
    private static final double STICK_EXPONENT = 2.0;
    // Wheel power limits for each speed mode (1.0 = full power).
    private static final double NORMAL_MAX_POWER = 1.0;
    private static final double PRECISION_MAX_POWER = 0.4;
    private static final double NUDGE_POWER = 0.3;
    private static final double WHEEL_TEST_POWER = 0.3;
    // Mecanum wheels slip more sideways than forward, so strafing is boosted a little to keep
    // diagonal moves on the angle the driver asked for. Use 1.0 to turn this off.
    private static final double STRAFE_CORRECTION = 1.1;

    // ---------- Intake. Powers 0 to 1; positive pulls balls in. ----------
    private static final double INTAKE_IN_POWER = 1.0;
    private static final double INTAKE_OUT_POWER = 0.7;

    // ---------- Shooter. ----------
    // Starting flywheel speed as a fraction of the motor's top speed, so it suits whatever motor
    // model the configuration names: 1.0 is the motor's full speed, 0.5 is half of it. Full
    // speed launches the balls as high as the motor can. For steadier shots use about 0.85: the
    // speed controller then has power in reserve to hold the speed when a ball loads the wheel
    // or the battery sags. LB + d-pad changes the speed by FLYWHEEL_RPM_STEP while running.
    private static final double FLYWHEEL_DEFAULT_SPEED = 1.0;
    private static final double FLYWHEEL_RPM_STEP = 100.0;
    // The flywheel is ready to shoot once its speed is within this fraction of the target. A
    // target above what the controller can hold counts as reached at the speed it can hold.
    private static final double FLYWHEEL_READY_TOLERANCE = 0.05;
    // Siphon servo power, 0 to 1.
    private static final double SIPHON_POWER = 1.0;

    // ---------- Hardware. ----------
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotor intakeMotor;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private DcMotorEx flywheel;
    private CRServo siphon;
    private double flywheelTicksPerRev;
    private double flywheelMaxRpm;
    private double flywheelAchievableRpm;
    private String flywheelMotorModel;
    private boolean flywheelModelUnspecified;

    // ---------- State. ----------
    private boolean flywheelOn = false;
    private double flywheelTargetRpm;  // set from the motor model in initHardware()
    private boolean siphonOn = false;

    // The last powers sent to each device, for telemetry.
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double intakeMotorPower;
    private double leftFeederPower;
    private double rightFeederPower;
    private double siphonPower;

    @Override
    public void runOpMode() {
        initHardware();

        while (opModeInInit()) {
            handleFlywheelSpeedButtons();
            // Button presses are remembered until read, so a Y or A press during INIT would
            // otherwise start the flywheel or siphon the moment the match starts. Read and
            // ignore them.
            gamepad1.yWasPressed();
            gamepad2.yWasPressed();
            gamepad1.aWasPressed();
            gamepad2.aWasPressed();

            addStickTelemetry();
            addShooterTelemetry();
            telemetry.addLine();
            addPortTelemetry();
            telemetry.addLine();
            telemetry.addLine("Left stick: move    Right stick: turn    Hold RB: precision");
            telemetry.addLine("D-pad: nudge    Hold Back + d-pad: run one wheel (wheel test)");
            telemetry.addLine("RT/LT: intake in/out    Y: flywheel on/off    A: siphon on/off");
            telemetry.addLine("B: carousel backward    LB + d-pad: flywheel speed");
            telemetry.update();
        }

        while (opModeIsActive()) {
            handleFlywheelSpeedButtons();

            // Read both gamepads every loop so neither one's press is left waiting.
            boolean y1 = gamepad1.yWasPressed();
            boolean y2 = gamepad2.yWasPressed();
            if (y1 || y2) {
                toggleFlywheel();
            }
            boolean a1 = gamepad1.aWasPressed();
            boolean a2 = gamepad2.aWasPressed();
            if (a1 || a2) {
                siphonOn = !siphonOn;
            }

            drive();
            runIntake();
            runCarousel();

            addStickTelemetry();
            addShooterTelemetry();
            addIntakeTelemetry();
            addDriveTelemetry();
            telemetry.update();
        }

        // STOP was pressed: leave everything stopped.
        setWheelPowers(0.0, 0.0, 0.0, 0.0);
        setIntakePowers(0.0, 0.0, 0.0);
        stopFlywheel();
        runSiphon(0.0);
    }

    // =========================================================================================
    // HARDWARE SETUP
    // =========================================================================================

    /** Finds every device by name and sets its direction and mode. */
    private void initHardware() {
        // ----- Drive motors -----
        frontLeft = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

        // The left and right motors face opposite ways, so one side is reversed to make positive
        // power roll every wheel forward. Use the wheel test (hold Back + d-pad) to check: any
        // wheel that rolls the robot BACKWARD during its test needs FORWARD/REVERSE flipped here.
        // If a DIFFERENT wheel moves than the one named on the screen, the names are on the wrong
        // ports in the Robot Configuration; fix that there, not here.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            // BRAKE stops the robot quickly when the sticks are released, instead of coasting.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // RUN_USING_ENCODER holds speeds more evenly, but only use it once all four drive
            // encoders are plugged in: a motor with no encoder signal runs at full power.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ----- Intake -----
        intakeMotor = hardwareMap.get(DcMotor.class, INTAKE_MOTOR_NAME);
        leftFeeder = hardwareMap.get(CRServo.class, LEFT_FEEDER_NAME);
        rightFeeder = hardwareMap.get(CRServo.class, RIGHT_FEEDER_NAME);

        // Positive power must pull balls IN on all three. The two feeders face each other across
        // the intake, so one of them is reversed. Flip FORWARD/REVERSE on anything that pushes
        // balls out instead.
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // FLOAT lets the intake coast to a stop instead of jerking a half-captured ball.
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ----- Shooter -----
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        siphon = hardwareMap.get(CRServo.class, SIPHON_NAME);

        // Positive flywheel speed must launch balls OUT of the shooter, and positive siphon power
        // must move balls toward the flywheel. Flip FORWARD/REVERSE on anything that runs the
        // wrong way.
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        siphon.setDirection(DcMotorSimple.Direction.FORWARD);

        // FLOAT lets the flywheel coast to a stop instead of braking hard, which is easier on the
        // gears. The run mode is switched in startFlywheel() and stopFlywheel(); see there.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Encoder ticks per turn and top speed come from the motor model in the configuration.
        MotorConfigurationType motorType = flywheel.getMotorType();
        flywheelTicksPerRev = motorType.getTicksPerRev();
        flywheelMaxRpm = motorType.getMaxRPM();
        flywheelMotorModel = motorType.getName();
        // "Unspecified Motor" stands in for a slow 165 RPM motor, so with it every RPM figure is
        // wrong. The flywheel then just runs flat out, and the INIT screen says so.
        flywheelModelUnspecified = flywheelMotorModel.equals(
                MotorConfigurationType.getUnspecifiedMotorType().getName());
        // The fastest speed the controller can really hold, from the model (85% of top speed for
        // most motors). Above this it is simply running at full power.
        flywheelAchievableRpm = flywheelMaxRpm * motorType.getAchieveableMaxRPMFraction();
        flywheelTargetRpm = Range.clip(FLYWHEEL_DEFAULT_SPEED * flywheelMaxRpm, 0.0, flywheelMaxRpm);
    }

    // =========================================================================================
    // DRIVING
    // =========================================================================================

    /** Turns either gamepad's sticks, bumper and d-pad into wheel powers. */
    private void drive() {
        // Wheel test takes over everything while Back is held.
        if (gamepad1.back || gamepad2.back) {
            runWheelTest();
            return;
        }

        // Left stick: where to move. Stick Y reads negative when pushed forward, so flip it.
        double stickX = stick(gamepad1.left_stick_x, gamepad2.left_stick_x);
        double stickY = -stick(gamepad1.left_stick_y, gamepad2.left_stick_y);

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

        // Right stick X: how fast to turn. Independent of the left stick, so on its own it spins
        // the robot in place, and together with the left stick it turns while moving.
        double turn = shape(stick(gamepad1.right_stick_x, gamepad2.right_stick_x));

        boolean precision = gamepad1.right_bumper || gamepad2.right_bumper;
        // With the left bumper held, that pad's d-pad sets the flywheel speed instead of nudging.
        Gamepad nudgePad = nudgingOn(gamepad1) ? gamepad1 : nudgingOn(gamepad2) ? gamepad2 : null;
        boolean nudging = nudgePad != null;

        if (nudging) {
            double nudgeForward = (nudgePad.dpad_up ? 1.0 : 0.0) - (nudgePad.dpad_down ? 1.0 : 0.0);
            double nudgeStrafe = (nudgePad.dpad_right ? 1.0 : 0.0) - (nudgePad.dpad_left ? 1.0 : 0.0);
            driveMecanum(nudgeForward, nudgeStrafe, turn, NUDGE_POWER);
        } else {
            driveMecanum(forward, strafe, turn, precision ? PRECISION_MAX_POWER : NORMAL_MAX_POWER);
        }

        telemetry.addData("Speed", nudging ? "Nudge" : precision ? "Precision" : "Normal");
    }

    /**
     * Mecanum drive relative to the robot. Each wheel's rollers push at 45 degrees, so the three
     * motions simply add up per wheel:
     *
     *   forward: all four wheels the same way
     *   strafe right: front left and back right forward, front right and back left backward
     *   turn clockwise: left side forward, right side backward
     *
     * @param forward  -1 to 1, positive drives forward
     * @param strafe   -1 to 1, positive slides right
     * @param turn     -1 to 1, positive rotates clockwise
     * @param maxPower 0 to 1, scales the whole motion (1 = full speed)
     */
    private void driveMecanum(double forward, double strafe, double turn, double maxPower) {
        strafe *= STRAFE_CORRECTION;

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // If any wheel would go over full power, scale all four down together. Clipping just the
        // big ones would change the ratios between wheels and send the robot the wrong way.
        double largest = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        double scale = maxPower / Math.max(largest, 1.0);

        setWheelPowers(fl * scale, fr * scale, bl * scale, br * scale);
    }

    /**
     * Wheel test: while Back is held, a d-pad button runs ONE wheel slowly forward. With the robot
     * on blocks, that wheel and only that wheel must turn, in the direction that would roll the
     * robot forward. Another wheel turning means the names are on the wrong ports in the Robot
     * Configuration; the right wheel turning backward means its direction in initHardware() must
     * be flipped. Driving that "moves when it should only turn" is almost always one of these.
     */
    private void runWheelTest() {
        boolean testFrontLeft = gamepad1.dpad_up || gamepad2.dpad_up;
        boolean testFrontRight = gamepad1.dpad_right || gamepad2.dpad_right;
        boolean testBackLeft = gamepad1.dpad_left || gamepad2.dpad_left;
        boolean testBackRight = gamepad1.dpad_down || gamepad2.dpad_down;

        setWheelPowers(testFrontLeft ? WHEEL_TEST_POWER : 0.0,
                testFrontRight ? WHEEL_TEST_POWER : 0.0,
                testBackLeft ? WHEEL_TEST_POWER : 0.0,
                testBackRight ? WHEEL_TEST_POWER : 0.0);

        String wheel = testFrontLeft ? "FRONT LEFT (topleft)" : testFrontRight ? "FRONT RIGHT (topright)"
                : testBackLeft ? "BACK LEFT (bottomleft)" : testBackRight ? "BACK RIGHT (bottomright)"
                : "press d-pad: up=front left  right=front right  left=back left  down=back right";
        telemetry.addData("WHEEL TEST", wheel);
        if (testFrontLeft || testFrontRight || testBackLeft || testBackRight) {
            telemetry.addLine("Only that wheel should turn, rolling the robot FORWARD.");
        }

        // Each wheel gets exactly the same power in this test, so a wheel that behaves
        // differently has a hardware problem. Run a good wheel, then the weak one, and compare
        // these numbers. (Only read here, not while driving, because each reading costs a few
        // milliseconds of loop time.)
        telemetry.addData("Amps", "FL %.2f   FR %.2f   BL %.2f   BR %.2f",
                frontLeft.getCurrent(CurrentUnit.AMPS), frontRight.getCurrent(CurrentUnit.AMPS),
                backLeft.getCurrent(CurrentUnit.AMPS), backRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Speed (ticks/s)", "FL %.0f   FR %.0f   BL %.0f   BR %.0f",
                frontLeft.getVelocity(), frontRight.getVelocity(),
                backLeft.getVelocity(), backRight.getVelocity());
        telemetry.addLine("Weak wheel vs a good wheel:");
        telemetry.addLine("  MORE amps, less speed  = something is binding or rubbing");
        telemetry.addLine("  LESS amps, less speed  = bad motor cable, plug or hub port");
        telemetry.addLine("  SAME speed, wheel lags = wheel hub / set screw slipping on the shaft");
        telemetry.addLine("  Speed 0 while turning  = that wheel's encoder cable is not plugged in");
    }

    /** Sets each wheel's power directly, -1 to 1. Positive rolls that wheel forward. */
    private void setWheelPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        frontLeftPower = fl;
        frontRightPower = fr;
        backLeftPower = bl;
        backRightPower = br;
    }

    // =========================================================================================
    // INTAKE
    // =========================================================================================

    /**
     * Right trigger pulls balls in, left trigger pushes them out, from either gamepad. In wins if
     * both are held. The feeder servos keep pulling balls in when neither is held.
     */
    private void runIntake() {
        boolean in = gamepad1.right_trigger_pressed || gamepad2.right_trigger_pressed;
        boolean out = gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed;
        if (in) {
            setIntakePowers(INTAKE_IN_POWER, INTAKE_IN_POWER, INTAKE_IN_POWER);
        } else if (out) {
            setIntakePowers(-INTAKE_OUT_POWER, -INTAKE_OUT_POWER, -INTAKE_OUT_POWER);
        } else {
            setIntakePowers(0.0, INTAKE_IN_POWER, INTAKE_IN_POWER);  // feeders only
        }
    }

    /** Runs the intake motor and each feeder, -1 to 1. Positive pulls balls in. */
    private void setIntakePowers(double motorPower, double leftPower, double rightPower) {
        intakeMotor.setPower(motorPower);
        leftFeeder.setPower(leftPower);
        rightFeeder.setPower(rightPower);

        intakeMotorPower = motorPower;
        leftFeederPower = leftPower;
        rightFeederPower = rightPower;
    }

    // =========================================================================================
    // SHOOTER
    // =========================================================================================

    /** Spins the flywheel up to the target speed and holds it there. */
    private void startFlywheel() {
        if (!flywheelOn) {
            flywheelOn = true;
            // The speed controller only works in RUN_USING_ENCODER mode.
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        flywheel.setVelocity(rpmToTicksPerSecond(flywheelTargetRpm));
    }

    /** Lets the flywheel coast to a stop. */
    private void stopFlywheel() {
        flywheelOn = false;
        // In RUN_USING_ENCODER mode, zero power means "hold a speed of zero", which brakes the
        // flywheel hard. Leaving that mode first makes zero power really zero, so it coasts.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0.0);
    }

    private void toggleFlywheel() {
        if (flywheelOn) {
            stopFlywheel();
        } else {
            startFlywheel();
        }
    }

    /** Changes the target speed by delta RPM (negative slows it), within what the motor can do. */
    private void adjustFlywheelRpm(double delta) {
        flywheelTargetRpm = Range.clip(flywheelTargetRpm + delta, 0.0, flywheelMaxRpm);
        if (flywheelOn) {
            flywheel.setVelocity(rpmToTicksPerSecond(flywheelTargetRpm));
        }
    }

    /** The flywheel's measured speed in RPM, from the encoder. */
    private double getFlywheelRpm() {
        return flywheel.getVelocity() * 60.0 / flywheelTicksPerRev;
    }

    /** True when the flywheel is on and close enough to the target speed to shoot. */
    private boolean flywheelReady(double rpm) {
        if (!flywheelOn || flywheelTargetRpm <= 0.0) {
            return false;
        }
        // A target above the speed the controller can hold is as reached as it will ever be
        // once the wheel passes that speed, so don't wait forever for it.
        double reachable = Math.min(flywheelTargetRpm, flywheelAchievableRpm);
        return rpm >= reachable * (1.0 - FLYWHEEL_READY_TOLERANCE)
                && rpm <= flywheelTargetRpm * (1.0 + FLYWHEEL_READY_TOLERANCE);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm / 60.0 * flywheelTicksPerRev;
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
                adjustFlywheelRpm(FLYWHEEL_RPM_STEP);
            }
            if (down) {
                adjustFlywheelRpm(-FLYWHEEL_RPM_STEP);
            }
        }
    }

    /** A switches the siphon on and off. Holding B turns the carousel backward to clear a jam. */
    private void runCarousel() {
        if (gamepad1.b || gamepad2.b) {
            runSiphon(-SIPHON_POWER);
        } else if (siphonOn) {
            runSiphon(SIPHON_POWER);
        } else {
            runSiphon(0.0);
        }
    }

    /** Runs the siphon servo directly: -1 to 1, positive moves balls toward the flywheel. */
    private void runSiphon(double power) {
        siphon.setPower(power);
        siphonPower = power;
    }

    // =========================================================================================
    // TELEMETRY
    // =========================================================================================

    private void addStickTelemetry() {
        // Raw stick readings from both pads. If a stick is being moved and its numbers stay at
        // 0.00, that controller is not registered with the Driver Station: press Start+A or
        // Start+B on it.
        telemetry.addData("Sticks", "GP1 L(%+.2f,%+.2f) R(%+.2f)   GP2 L(%+.2f,%+.2f) R(%+.2f)",
                gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x,
                gamepad2.left_stick_x, -gamepad2.left_stick_y, gamepad2.right_stick_x);
    }

    private void addShooterTelemetry() {
        double rpm = getFlywheelRpm();
        String state = !flywheelOn ? "OFF" : flywheelReady(rpm) ? "READY" : "SPINNING UP";
        telemetry.addData("Shooter", "%s   %.0f / %.0f rpm", state, rpm, flywheelTargetRpm);
        telemetry.addData("Siphon", siphonPower > 0.0 ? "Feeding" : siphonPower < 0.0 ? "Reverse" : "Stopped");
        if (flywheelOn && rpm == 0.0) {
            telemetry.addLine("Flywheel reads 0 rpm: check its encoder cable and motor model");
        }
        if (flywheelModelUnspecified) {
            telemetry.addLine("Turret motor model not set: rpm figures are wrong, flywheel runs flat out");
        }
    }

    private void addIntakeTelemetry() {
        String state = intakeMotorPower > 0.0 ? "In" : intakeMotorPower < 0.0 ? "Out"
                : leftFeederPower != 0.0 || rightFeederPower != 0.0 ? "Feeders only" : "Stopped";
        telemetry.addData("Intake", "%s  (motor %4.2f, feeders %4.2f / %4.2f)",
                state, intakeMotorPower, leftFeederPower, rightFeederPower);
    }

    private void addDriveTelemetry() {
        telemetry.addData("Front L / R", "%5.2f  %5.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  L / R", "%5.2f  %5.2f", backLeftPower, backRightPower);
    }

    /** The port each name was found on, to compare with the table at the top of this file. */
    private void addPortTelemetry() {
        telemetry.addData("Drive ports", "topleft %d  topright %d  bottomleft %d  bottomright %d",
                frontLeft.getPortNumber(), frontRight.getPortNumber(),
                backLeft.getPortNumber(), backRight.getPortNumber());
        telemetry.addData("Intake ports", "mainmotorintake %d  leftfeeder %d  rightfeeder %d",
                intakeMotor.getPortNumber(), leftFeeder.getPortNumber(), rightFeeder.getPortNumber());
        telemetry.addData("Shooter ports", "turret %d  servosiphon %d",
                flywheel.getPortNumber(), siphon.getPortNumber());
        telemetry.addData("Flywheel motor model", "%s: %.0f ticks/turn, %.0f RPM max",
                flywheelMotorModel, flywheelTicksPerRev, flywheelMaxRpm);
        if (flywheelModelUnspecified) {
            telemetry.addLine("TURRET MOTOR MODEL NOT SET: the rpm figures are wrong and the flywheel");
            telemetry.addLine("just runs flat out. Pick its real model in the Robot Configuration.");
        }
    }

    // =========================================================================================
    // GAMEPAD HELPERS
    // =========================================================================================

    /** True while this pad's d-pad is nudging: a d-pad button held without the left bumper. */
    private static boolean nudgingOn(Gamepad pad) {
        return !pad.left_bumper && (pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right);
    }

    /**
     * Merges one stick axis from both gamepads: whichever pad is pushed further wins. This is what
     * lets either pad drive. It also means a controller the Driver Station has registered as
     * user 2 (Start+B) still drives, instead of being ignored by code that only reads gamepad1.
     */
    private static double stick(float pad1Value, float pad2Value) {
        return Math.abs(pad1Value) >= Math.abs(pad2Value) ? pad1Value : pad2Value;
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
