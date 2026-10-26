package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Odometry test. Drives exactly like TeleOp (left stick drives and strafes, right stick
 * turns, either gamepad) and shows where the robot thinks it is. Position comes from the four
 * wheel encoders; heading comes from the Control Hub IMU, with the encoder heading shown next
 * to it as a check. Nothing else runs: no intake, no flywheel.
 *
 * PICK THE ALLIANCE DURING INIT (either gamepad): top button (Y / triangle) = BLUE, right
 * button (B / circle) = RED. The position then starts at that alliance's start spot, the same
 * one Auto2 uses: backward, front (intake) pressed against the wall behind your HIVE, centered on
 * the middle tile in front of your goal, launcher facing your upward CELL.
 *   RED    tile C1: x 60, y 8.5, front facing the audience wall (heading -90)
 *   BLUE   tile D6: x 84, y 135.5, front facing the back wall (heading 90)
 *
 * FIRST drive a little with the sticks: that is how it finds out which way each wheel's encoder
 * counts (the back right one counts backward because that motor is wired the other way) and
 * whether one isn't counting at all. The "Wheel" lines say "?" until then.
 *
 * Position is on the real field, in inches, seen from the audience:
 *   X          from the RED wall toward the BLUE wall (0 to 144)
 *   Y          from the audience wall toward the back wall (0 to 144)
 *   Heading    degrees: 0 = facing the BLUE wall, 90 = the back wall, 180 = the RED wall
 *
 *   Triangle (Y)   put the robot back on its start spot and press: the position starts over
 *
 * MAP: the whole field from above, audience at the bottom, RED wall on the left. S is the start,
 * the arrow is the robot pointing where it faces (> ^ < v), dots are where it has been, r and b
 * are the red and blue HIVE CELLS, L are the LOADING ZONES, and there is a + in the corner of
 * every tile. One column is 6 in., one row is 12 in. (text is about twice as tall as wide).
 * Each wheel's distance is shown in inches too, next to its encoder count.
 *
 * How to check it (after driving a little): put it back on the start spot, press triangle, then
 * drive it 24 inches toward the BLUE wall: X should go up by 24. 24 inches toward the back wall:
 * Y should go up by 24. A quarter turn to the left: the heading should go up by 90. If a distance
 * is off by a constant factor, fix WHEEL_DIAMETER_MM or TICKS_PER_REV below. A wheel that says
 * NOT COUNTING has its encoder cable unplugged or in the wrong port.
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

    // Wheel directions, same as TeleOp. A wheel that rolls forward must count UP.
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

    // RED start spot, inches: the middle of tile C1, front against the audience wall, so y is half
    // the robot's length. BLUE's spot is this one turned half a turn about the middle of the field.
    private static final double ROBOT_LENGTH_IN = 17.0;
    private static final double RED_START_X = 60.0;
    private static final double RED_START_Y = ROBOT_LENGTH_IN / 2.0;
    private static final double FIELD_IN = 144.0;

    // The map: the whole field, 24 columns of 6 in. and 12 rows of 12 in.
    private static final int MAP_COLS = 24;
    private static final int MAP_ROWS = 12;
    private static final double IN_PER_COL = 6.0;
    private static final double IN_PER_ROW = 12.0;

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
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);  // all encoders read together, faster
        }
        DcMotor[] wheels = {topLeft, topRight, bottomLeft, bottomRight};
        for (DcMotor wheel : wheels) {
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // counts start at 0
            wheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);     // drive feels like TeleOp
        }

        // tryGet returns null instead of crashing if the configuration has no "imu".
        IMU imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu != null) {
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        }

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);  // so the map lines up

        // INIT: pick the alliance. Top button = BLUE, right button = RED, either gamepad.
        boolean blue = false;
        while (opModeInInit()) {
            if (gamepad1.y || gamepad2.y) blue = true;
            if (gamepad1.b || gamepad2.b) blue = false;
            telemetry.addLine("=====  ALLIANCE: " + (blue ? "BLUE" : "RED") + "  =====");
            telemetry.addLine("Top button (Y / triangle) = BLUE.  Right button (B / circle) = RED.");
            telemetry.addLine("Start spot: backward, front against the wall behind your goal, middle tile ("
                    + (blue ? "BLUE D6" : "RED C1") + "). Then START and drive a little first.");
            telemetry.addData("IMU", imu != null ? "found" : "NOT in configuration, heading will come from the encoders");
            telemetry.update();
        }
        if (imu != null) {
            imu.resetYaw();
        }

        // The start spot on the field: inches and radians. BLUE's is RED's turned half a turn.
        double startX = blue ? FIELD_IN - RED_START_X : RED_START_X;
        double startY = blue ? FIELD_IN - RED_START_Y : RED_START_Y;
        // Front facing the wall behind the HIVE: RED faces the audience wall, BLUE the back wall.
        double startHeading = blue ? Math.PI / 2 : -Math.PI / 2;
        int[] startCell = mapCell(startX, startY);

        // Where the robot is on the field: mm from the RED wall, mm from the audience wall,
        // radians (0 = facing the BLUE wall, counter-clockwise is positive).
        double x = startX * MM_PER_INCH;
        double y = startY * MM_PER_INCH;
        double heading = startHeading;
        double encoderHeading = 0.0;  // turn since the start from the wheels alone, for comparison
        int[] prev = new int[4];
        for (int i = 0; i < 4; i++) {
            prev[i] = wheels[i].getCurrentPosition();
        }
        // Each wheel: power sent x ticks counted, added up. Positive = its encoder counts the way
        // the wheel is driven; negative = it counts backward, so its counts get flipped.
        double[] evidence = new double[4];
        double[] sign = {1, 1, 1, 1};
        double[] counted = new double[4];
        boolean[] dead = new boolean[4];
        double[] lastPower = new double[4];
        boolean[][] path = new boolean[MAP_ROWS][MAP_COLS];  // map squares the robot has been in

        while (opModeIsActive()) {
            // Driving, copied from TeleOp. Whichever gamepad is pushed further drives.
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

            // Odometry: how far each wheel rolled since the last loop, in mm. Until a wheel has
            // been driven enough to know which way its encoder counts, assume it turned the way
            // it was driven, so a backward encoder can't fake a sideways slide.
            int[] ticks = new int[4];
            double[] d = new double[4];
            for (int i = 0; i < 4; i++) {
                ticks[i] = wheels[i].getCurrentPosition();
                int moved = ticks[i] - prev[i];
                prev[i] = ticks[i];
                evidence[i] += lastPower[i] * moved;
                boolean known = Math.abs(evidence[i]) > 30;
                if (known) {
                    sign[i] = Math.signum(evidence[i]);
                }
                d[i] = (known || lastPower[i] == 0 ? moved * sign[i] : Math.abs(moved) * Math.signum(lastPower[i])) * MM_PER_TICK;
                counted[i] += Math.abs(moved);
            }
            lastPower = new double[] {fl, fr, bl, br};
            // An encoder that hardly counts while the others have rolled a couple of inches isn't
            // working; its wheel is worked out from the other three (front pair = back pair).
            double total = counted[0] + counted[1] + counted[2] + counted[3];
            for (int i = 0; i < 4; i++) {
                double others = (total - counted[i]) / 3;
                dead[i] = others > 100 && counted[i] < 0.1 * others;
            }
            if (dead[0]) d[0] = d[2] + d[3] - d[1];
            if (dead[1]) d[1] = d[2] + d[3] - d[0];
            if (dead[2]) d[2] = d[0] + d[1] - d[3];
            if (dead[3]) d[3] = d[0] + d[1] - d[2];
            double dFL = d[0];
            double dFR = d[1];
            double dBL = d[2];
            double dBR = d[3];

            // Mecanum forward kinematics: the reverse of the fl/fr/bl/br formulas above.
            double dForward = (dFL + dFR + dBL + dBR) / 4.0;
            double dLeft = (-dFL + dFR + dBL - dBR) / 4.0;
            encoderHeading += (-dFL + dFR - dBL + dBR) / (2.0 * (TRACK_WIDTH_MM + WHEEL_BASE_MM));

            double newHeading = startHeading + (imu != null
                    ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
                    : encoderHeading);

            // Turn the robot-relative move into a field-relative one, using the heading halfway
            // through the move so turning while driving does not bend the path.
            double midHeading = heading + wrap(newHeading - heading) / 2.0;
            x += dForward * Math.cos(midHeading) - dLeft * Math.sin(midHeading);
            y += dForward * Math.sin(midHeading) + dLeft * Math.cos(midHeading);
            heading = newHeading;

            if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) {  // back on the start spot
                x = startX * MM_PER_INCH;
                y = startY * MM_PER_INCH;
                heading = startHeading;
                encoderHeading = 0.0;
                path = new boolean[MAP_ROWS][MAP_COLS];
                if (imu != null) {
                    imu.resetYaw();
                }
            }

            double xIn = x / MM_PER_INCH;
            double yIn = y / MM_PER_INCH;
            int[] cell = mapCell(xIn, yIn);
            path[cell[0]][cell[1]] = true;

            telemetry.addData("Alliance", blue ? "BLUE" : "RED");
            telemetry.addData("Position", "X %.1f in from RED wall   Y %.1f in from audience", xIn, yIn);
            telemetry.addData("Heading", "%+.1f deg, 0 = facing BLUE wall   (encoders say %+.1f)",
                    Math.toDegrees(wrap(heading)), Math.toDegrees(wrap(startHeading + encoderHeading)));
            showMap(cell, startCell, Math.toDegrees(wrap(heading)), path);
            telemetry.addData("Heading from", imu != null ? "IMU" : "encoders only (no IMU)");
            String[] names = {"FL", "FR", "BL", "BR"};
            for (int i = 0; i < 4; i++) {
                String status = dead[i] ? "NOT COUNTING (cable?)" : Math.abs(evidence[i]) <= 30 ? "? drive a little"
                        : sign[i] < 0 ? "counts BACKWARD (fixed)" : "ok";
                telemetry.addData("Wheel " + names[i], "%+7.1f in  %6d ticks  %s", inches(ticks[i]) * sign[i], ticks[i], status);
            }
            telemetry.addLine("Triangle = robot is back on the start spot, start over");
            telemetry.update();
        }
    }

    /** How far a wheel has rolled since INIT, in inches, from its encoder count. */
    private static double inches(int ticks) {
        return ticks * MM_PER_TICK / MM_PER_INCH;
    }

    /** The map square {row, column} for a field position in inches; off the field it stays on the edge. */
    private static int[] mapCell(double xIn, double yIn) {
        int row = MAP_ROWS - 1 - (int) Math.floor(yIn / IN_PER_ROW);  // back wall at the top
        int col = (int) Math.floor(xIn / IN_PER_COL);                 // RED wall on the left
        return new int[] {Range.clip(row, 0, MAP_ROWS - 1), Range.clip(col, 0, MAP_COLS - 1)};
    }

    /** Draws the whole field: tiles, HIVE CELLS, LOADING ZONES, the path, the start and the robot. */
    private void showMap(int[] robot, int[] start, double headingDeg, boolean[][] path) {
        char[][] map = new char[MAP_ROWS][MAP_COLS];
        for (int r = 0; r < MAP_ROWS; r++) {
            for (int c = 0; c < MAP_COLS; c++) {
                map[r][c] = (r % 2 == 0 && c % 4 == 0) ? '+' : ' ';  // corner of each tile
            }
        }
        mark(map, 59, 62, 'r');   // red HIVE CELLS
        mark(map, 59, 82, 'r');
        mark(map, 85, 62, 'b');   // blue HIVE CELLS
        mark(map, 85, 82, 'b');
        mark(map, 3, 102, 'L');   // red LOADING ZONE
        mark(map, 3, 114, 'L');
        mark(map, 141, 30, 'L');  // blue LOADING ZONE
        mark(map, 141, 42, 'L');
        for (int r = 0; r < MAP_ROWS; r++) {
            for (int c = 0; c < MAP_COLS; c++) {
                if (path[r][c]) map[r][c] = '.';
            }
        }
        map[start[0]][start[1]] = 'S';
        map[robot[0]][robot[1]] = Math.abs(headingDeg) <= 45.0 ? '>'
                : Math.abs(headingDeg) >= 135.0 ? '<'
                : headingDeg > 0.0 ? '^' : 'v';

        StringBuilder border = new StringBuilder("+");
        for (int c = 0; c < MAP_COLS; c++) {
            border.append('-');
        }
        border.append('+');
        telemetry.addLine(border.toString());
        for (char[] row : map) {
            telemetry.addLine("|" + new String(row) + "|");
        }
        telemetry.addLine(border.toString());
        telemetry.addLine("RED wall left, BLUE right, audience at the bottom");
        telemetry.addLine("S start  arrow robot  . path  r b CELLS  L LOADING ZONE");
    }

    private static void mark(char[][] map, double xIn, double yIn, char symbol) {
        int[] cell = mapCell(xIn, yIn);
        map[cell[0]][cell[1]] = symbol;
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
    