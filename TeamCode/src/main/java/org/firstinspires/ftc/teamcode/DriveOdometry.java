package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Works out where the robot is from how far its four drive wheels have turned, plus the Control
 * Hub's gyro for which way it is facing. Call update() once every time round the OpMode's loop.
 *
 * Positions are in inches, measured from the spot the robot started on (or was last reset on):
 *   x   how far FORWARD of that spot, "forward" being the way the robot faced at the start
 *   y   how far to the LEFT of that spot
 * The heading is in degrees, 0 at the start, growing as the robot turns left.
 *
 * What it needs:
 *   - all four drive motors' ENCODER cables plugged into the Control Hub, next to their power
 *     cables. A wheel with no encoder cable reads 0 forever and the position comes out wrong;
 *     telemetry shows the four raw counts so a dead one is easy to spot.
 *   - every wheel's direction set so positive power rolls it forward (the TeleOp wheel test).
 *   - the hub's gyro in the Robot Configuration as "imu". Without it the heading is worked out
 *     from the wheels too, which is much less accurate.
 *
 * How far to trust it: this is the roughest kind of position tracking. Mecanum wheels slip,
 * most of all when strafing, pushing against something or being bumped, and every slip is
 * counted as movement. Expect it to be a few inches out after a short drive and to get worse
 * through a match. Putting the robot back on its starting spot and resetting brings it back.
 */
public class DriveOdometry {

    public static final String IMU_NAME = "imu";

    // Drive motor and wheel: goBILDA 5203 series 19.2:1 (537.7 encoder ticks per wheel turn)
    // with 104 mm mecanum wheels. CHECK THESE AGAINST THE ROBOT: if a measured 48 inch drive
    // reads long or short on telemetry, the wheel diameter is the number to correct.
    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_DIAMETER_IN = 104.0 / 25.4;

    // Mecanum wheels slip sideways, so the encoders count more than the robot really slides.
    // Strafe a measured distance and adjust this until telemetry agrees.
    public static final double STRAFE_SCALE = 0.9;

    // Only used when there is no gyro: half the distance between the left and right wheels plus
    // half the distance between the front and back wheels, in inches.
    public static final double TURN_LEVER_IN = 12.0;

    // How the Control Hub is mounted; the same values as MecanumDrivetrain.
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private static final double IN_PER_TICK = Math.PI * WHEEL_DIAMETER_IN / TICKS_PER_REV;

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final IMU imu;  // null if the configuration has no IMU

    private int lastFrontLeft;
    private int lastFrontRight;
    private int lastBackLeft;
    private int lastBackRight;
    private double gyroAtReset;  // radians

    private double x;        // inches forward of the start
    private double y;        // inches left of the start
    private double heading;  // radians, counter-clockwise from the start

    /** Pass the drive motors after their directions have been set. */
    public DriveOdometry(HardwareMap hardwareMap, DcMotor frontLeft, DcMotor frontRight,
                         DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // tryGet returns null instead of crashing, so the robot can still drive without an IMU.
        imu = hardwareMap.tryGet(IMU.class, IMU_NAME);
        if (imu != null) {
            imu.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(HUB_LOGO_DIRECTION, HUB_USB_DIRECTION)));
        }
        reset();
    }

    /** Says "the robot is on its starting spot, facing the starting way, right now". */
    public void reset() {
        lastFrontLeft = frontLeft.getCurrentPosition();
        lastFrontRight = frontRight.getCurrentPosition();
        lastBackLeft = backLeft.getCurrentPosition();
        lastBackRight = backRight.getCurrentPosition();
        gyroAtReset = readGyro();
        x = 0.0;
        y = 0.0;
        heading = 0.0;
    }

    /** Adds the movement since the last call. Call once every loop. */
    public void update() {
        int fl = frontLeft.getCurrentPosition();
        int fr = frontRight.getCurrentPosition();
        int bl = backLeft.getCurrentPosition();
        int br = backRight.getCurrentPosition();
        double dFl = (fl - lastFrontLeft) * IN_PER_TICK;
        double dFr = (fr - lastFrontRight) * IN_PER_TICK;
        double dBl = (bl - lastBackLeft) * IN_PER_TICK;
        double dBr = (br - lastBackRight) * IN_PER_TICK;
        lastFrontLeft = fl;
        lastFrontRight = fr;
        lastBackLeft = bl;
        lastBackRight = br;

        // The mecanum sums run backwards: the TeleOp makes each wheel's power from forward,
        // strafe and turn, and this recovers forward, strafe and turn from what each wheel did.
        double forward = (dFl + dFr + dBl + dBr) / 4.0;
        double left = -(dFl - dFr - dBl + dBr) / 4.0 * STRAFE_SCALE;

        double turned;  // radians counter-clockwise since the last call
        if (imu != null) {
            turned = wrap(readGyro() - gyroAtReset - heading);
        } else {
            turned = (-dFl + dFr - dBl + dBr) / 4.0 / TURN_LEVER_IN;
        }

        // The robot moved while it was turning, so use the heading halfway through the step to
        // turn "forward and left of the robot" into "forward and left of the starting spot".
        double mid = heading + turned / 2.0;
        x += forward * Math.cos(mid) - left * Math.sin(mid);
        y += forward * Math.sin(mid) + left * Math.cos(mid);
        heading += turned;
    }

    /** Inches forward of the starting spot. */
    public double getX() {
        return x;
    }

    /** Inches left of the starting spot. */
    public double getY() {
        return y;
    }

    /** Degrees turned left since the start, -180 to 180. */
    public double getHeadingDegrees() {
        return Math.toDegrees(wrap(heading));
    }

    /** Straight-line distance in inches from the robot to a point given the same way as x, y. */
    public double distanceTo(double pointX, double pointY) {
        return Math.hypot(pointX - x, pointY - y);
    }

    public boolean hasGyro() {
        return imu != null;
    }

    private double readGyro() {
        return imu == null ? 0.0 : imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Brings an angle in radians back into -pi..pi. */
    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /** Adds the position to telemetry. The caller still calls telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Position", "%.0f in forward  %.0f in left  facing %.0f deg%s",
                x, y, getHeadingDegrees(), imu == null ? "  (NO GYRO: heading from wheels)" : "");
        telemetry.addData("Wheel encoders", "FL %d  FR %d  BL %d  BR %d   (one stuck at 0 = cable missing)",
                lastFrontLeft, lastFrontRight, lastBackLeft, lastBackRight);
    }
}
