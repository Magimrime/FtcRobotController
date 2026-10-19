package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Odometry test. Drives exactly like RobotTeleOp (left stick drives and strafes, right stick
 * turns, either gamepad) and shows where the robot thinks it is. Position comes from the four
 * wheel encoders; heading comes from the Control Hub IMU, with the encoder heading shown next
 * to it as a check. Nothing else runs: no intake, no flywheel.
 *
 * Everything is measured from where the robot was at START (or when triangle was last pressed):
 *   X          how far forward it has gone, inches (negative = backward)
 *   Y          how far left it has gone, inches (negative = right)
 *   Heading    how far it has turned, degrees, positive = left (counter-clockwise)
 *
 *   Triangle (Y)   zero the position and heading
 *
 * How to check it: push the robot forward exactly 24 inches by hand and X should read 24.
 * Push it right 24 inches and Y should read -24. Turn it a quarter turn to the left and the
 * heading should read 90. If a distance is off by a constant factor, fix WHEEL_DIAMETER_MM or
 * TICKS_PER_REV below. If the encoder counts at the bottom of the screen stay at 0 while the
 * wheels turn, the encoder cables are not plugged into the Control Hub.
 *
 * Mecanum wheels slip sideways, so strafing reads a little short. That is normal for wheel
 * odometry; driving over a game element throws it off too.
 *
 * Robot Configuration names:
 *   topleft, topright, bottomleft, bottomright   wheel motors, Control Hub motor ports 0-3
 *   imu                                          the IMU built into the Control Hub
 */
@TeleOp(name = "Odometry Test", group = "Test")
public class OdometryTest extends LinearOpMode {

    // Wheel directions, same as RobotTeleOp. A wheel that rolls forward must count UP.
    private static final DcMotorSimple.Direction TOP_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction TOP_RIGHT = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction BOTTOM_LEFT = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction BOTTOM_RIGHT = DcMotorSimple.Direction.REVERSE;

    // Wheel and robot sizes. goBILDA 19.2:1 Yellow Jacket = 537.7 ticks per turn; the StarterBot
    // mecanum wheels are 96 mm across. Measure the last two on the robot: track width is from the
    // middle of a left wheel to the middle of a right wheel, wheel base is front axle to back
    // axle. Those two only affect the heading worked out from the encoders.
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_MM = 96.0;
    private static final double TRACK_WIDTH_MM = 350.0;  // approximate, measure it
    private static final double WHEEL_BASE_MM = 300.0;   // approximate, measure it
    private static final double MM_PER_TICK = Math.PI * WHEEL_DIAMETER_MM / TICKS_PER_REV;
    private static final double MM_PER_INCH = 25.4;

    // How the Control Hub sits on the robot: REV logo facing up, USB ports toward the front.
    // If the heading moves when the robot tilts instead of when it turns, these are wrong.
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    @Override
    public void runOpMode() {
        DcMotor topLeft = hardwareMap.get(DcMotor.class, "topleft");
        DcMotor topRight = hardwareMap.get(DcMotor.class, "topright");
        DcMotor bottomLeft = hardwareMap.get(DcMotor.class, "bottomleft");
        DcMotor bottomRight = hardwareMap.get(DcMotor.class, "bottomright");

        topLeft.setDirection(TOP_LEFT);
        topRight.setDirection(TOP_RIGHT);
        bottomLeft.setDirection(BOTTOM_LEFT);
        bottomRight.setDirection(BOTTOM_RIGHT);
        for (DcMotor wheel : new DcMotor[] {topLeft, topRight, bottomLeft, bottomRight}) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // counts start at 0
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);     // drive feels like RobotTeleOp
        }

        // tryGet returns null instead of crashing if the configuration has no "imu".
        IMU imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        }

        telemetry.addLine("Odometry Test. Position and heading are zeroed when you press START.");
        telemetry.addData("IMU", imu != null ? "found" : "NOT in configuration, heading will come from the encoders");
        telemetry.update();
        waitForStart();
        if (imu != null) {
            imu.resetYaw();
        }

        // Where the robot is, relative to the start: mm forward, mm left, radians left (CCW).
        double x = 0.0;
        double y = 0.0;
        double heading = 0.0;
        double encoderHeading = 0.0;  // heading from the wheels alone, for comparison
        int prevFL = topLeft.getCurrentPosition();
        int prevFR = topRight.getCurrentPosition();
        int prevBL = bottomLeft.getCurrentPosition();
        int prevBR = bottomRight.getCurrentPosition();

        while (opModeIsActive()) {
            // Driving, copied from RobotTeleOp. Whichever gamepad is pushed further drives.
            double forward = -stick(gamepad1.left_stick_y, gamepad2.left_stick_y);
            double strafe = stick(gamepad1.left_stick_x, gamepad2.left_stick_x);
            double turn = stick(gamepad1.right_stick_x, gamepad2.right_stick_x);

            double fl = forward + strafe + turn;
            double fr = forward - strafe - turn;
            double bl = forward - strafe + turn;
            double br = forward + strafe - turn;

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

            // Odometry: how far each wheel turned since the last loop, in mm.
            int ticksFL = topLeft.getCurrentPosition();
            int ticksFR = topRight.getCurrentPosition();
            int ticksBL = bottomLeft.getCurrentPosition();
            int ticksBR = bottomRight.getCurrentPosition();
            double dFL = (ticksFL - prevFL) * MM_PER_TICK;
            double dFR = (ticksFR - prevFR) * MM_PER_TICK;
            double dBL = (ticksBL - prevBL) * MM_PER_TICK;
            double dBR = (ticksBR - prevBR) * MM_PER_TICK;
            prevFL = ticksFL;
            prevFR = ticksFR;
            prevBL = ticksBL;
            prevBR = ticksBR;

            // Mecanum forward kinematics: the reverse of the fl/fr/bl/br formulas above.
            double dForward = (dFL + dFR + dBL + dBR) / 4.0;
            double dLeft = (-dFL + dFR + dBL - dBR) / 4.0;
            encoderHeading += (-dFL + dFR - dBL + dBR) / (2.0 * (TRACK_WIDTH_MM + WHEEL_BASE_MM));

            double newHeading = imu != null
                    ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                    : encoderHeading;

            // Turn the robot-relative move into a field-relative one, using the heading halfway
            // through the move so turning while driving does not bend the path.
            double midHeading = heading + wrap(newHeading - heading) / 2.0;
            x += dForward * Math.cos(midHeading) - dLeft * Math.sin(midHeading);
            y += dForward * Math.sin(midHeading) + dLeft * Math.cos(midHeading);
            heading = newHeading;

            if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) {
                x = 0.0;
                y = 0.0;
                heading = 0.0;
                encoderHeading = 0.0;
                if (imu != null) {
                    imu.resetYaw();
                }
            }

            telemetry.addData("Position", "X %+.1f in forward   Y %+.1f in left", x / MM_PER_INCH, y / MM_PER_INCH);
            telemetry.addData("Heading", "%+.1f deg left   (encoders say %+.1f)",
                    Math.toDegrees(wrap(heading)), Math.toDegrees(wrap(encoderHeading)));
            telemetry.addData("Heading from", imu != null ? "IMU" : "encoders only (no IMU)");
            telemetry.addData("Position mm", "X %+.0f   Y %+.0f", x, y);
            telemetry.addData("Encoders", "FL %d  FR %d  BL %d  BR %d", ticksFL, ticksFR, ticksBL, ticksBR);
            telemetry.addData("Wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", fl, fr, bl, br);
            telemetry.addLine("Triangle = zero position and heading");
            telemetry.update();
        }
    }

    /** One stick axis from both gamepads: whichever is pushed further wins. */
    private static double stick(float pad1, float pad2) {
        return Math.abs(pad1) >= Math.abs(pad2) ? pad1 : pad2;
    }

    /** Brings an angle in radians into -180..180 degrees, so -170 and +190 compare as the same. */
    private static double wrap(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
}
    