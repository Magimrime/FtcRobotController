package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Hardware and math for a 4-wheel mecanum drivetrain. This is not an OpMode: TeleOp and
 * Autonomous OpModes create one and tell it how to move, so the drive code lives in one place.
 *
 * Wheel layout viewed from above, with each wheel's configuration name and Control Hub motor
 * port. The intake is the front, so driving forward goes intake-first ("top" in the names means
 * the front, "bottom" the back). The rollers on top of the wheels must form an "X" like this,
 * otherwise strafing goes the wrong way.
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
 * Every drive method takes the same three motions, and any mix of them is allowed:
 *   forward   +1 = drive forward       -1 = drive backward
 *   strafe    +1 = slide right         -1 = slide left
 *   turn      +1 = rotate clockwise    -1 = rotate counter-clockwise
 */
public class MecanumDrivetrain {

    // Device names. The code finds motors by name, not port: in the Robot Configuration on the
    // Driver Station, give each motor port the name shown for it in the diagram above.
    public static final String FRONT_LEFT_NAME = "topleft";
    public static final String FRONT_RIGHT_NAME = "topright";
    public static final String BACK_LEFT_NAME = "bottomleft";
    public static final String BACK_RIGHT_NAME = "bottomright";
    public static final String IMU_NAME = "imu";

    // How the Control Hub is mounted. Field-centric driving is only correct if these match the
    // robot. The SDK sample ConceptExploringIMUOrientation helps work them out.
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Mecanum wheels slip more sideways than forward, so strafing is boosted a little to keep
    // diagonal moves on the angle the driver asked for. Use 1.0 to turn this off.
    public static final double STRAFE_CORRECTION = 1.1;

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;  // null if the configuration has no IMU

    // The last power sent to each wheel, for telemetry.
    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotor.class, FRONT_RIGHT_NAME);
        backLeft = hardwareMap.get(DcMotor.class, BACK_LEFT_NAME);
        backRight = hardwareMap.get(DcMotor.class, BACK_RIGHT_NAME);

        // The left and right motors face opposite ways, so one side is reversed to make positive
        // power roll every wheel forward. This suits motors that drive the wheels directly; extra
        // gears or right-angle drives can change it. Check with the "Mecanum Motor Test" utility
        // and flip FORWARD/REVERSE for any wheel that turns backward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            // BRAKE stops the robot quickly when the sticks are released, instead of coasting.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // RUN_USING_ENCODER holds speeds more evenly, but only use it once all four drive
            // encoders are plugged in: a motor with no encoder signal runs at full power.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // tryGet returns null instead of crashing, so the robot can still drive without an IMU.
        imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION)));
        }
    }

    /**
     * Drives relative to the robot: forward is the way the robot is facing.
     *
     * @param forward  -1 to 1, positive drives forward
     * @param strafe   -1 to 1, positive slides right
     * @param turn     -1 to 1, positive rotates clockwise
     * @param maxPower 0 to 1, scales the whole motion (1 = full speed)
     */
    public void drive(double forward, double strafe, double turn, double maxPower) {
        strafe *= STRAFE_CORRECTION;

        // Each wheel pushes diagonally, so the three motions combine by simple addition.
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
     * Drives relative to the field: forward is away from the driver, whichever way the robot is
     * facing. Parameters are the same as drive(). Without an IMU this behaves like drive().
     */
    public void driveFieldCentric(double forward, double strafe, double turn, double maxPower) {
        // Rotate the requested direction by minus the robot's heading, which turns a direction on
        // the field into the same direction measured from the robot. Turning needs no change.
        double heading = getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double robotForward = forward * cos - strafe * sin;
        double robotStrafe = forward * sin + strafe * cos;

        drive(robotForward, robotStrafe, turn, maxPower);
    }

    /** Sets each wheel's power directly, -1 to 1. Positive rolls that wheel forward. */
    public void setWheelPowers(double frontLeftPower, double frontRightPower,
                               double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        this.frontLeftPower = frontLeftPower;
        this.frontRightPower = frontRightPower;
        this.backLeftPower = backLeftPower;
        this.backRightPower = backRightPower;
    }

    /** True if an IMU was found. Field-centric driving needs one. */
    public boolean hasImu() {
        return imu != null;
    }

    /**
     * The robot's heading: 0 at the last resetHeading(), increasing as the robot turns
     * counter-clockwise (left). Always 0 without an IMU.
     */
    public double getHeading(AngleUnit angleUnit) {
        if (imu == null) {
            return 0.0;
        }
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    /** Makes the direction the robot faces now heading 0, which field-centric calls forward. */
    public void resetHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }

    /** Adds the port each wheel is configured on, to compare with the diagram at the top. */
    public void addPortTelemetry(Telemetry telemetry) {
        telemetry.addData("Ports", "FL %d   FR %d   BL %d   BR %d",
                frontLeft.getPortNumber(), frontRight.getPortNumber(),
                backLeft.getPortNumber(), backRight.getPortNumber());
    }

    /** Adds the last wheel powers to telemetry. The caller still calls telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Front L / R", "%5.2f  %5.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  L / R", "%5.2f  %5.2f", backLeftPower, backRightPower);
    }
}
