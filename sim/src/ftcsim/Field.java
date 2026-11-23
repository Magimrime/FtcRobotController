package ftcsim;

/**
 * The BIOBUZZ field as the simulator sees it, in FIELD inches as the audience sees it:
 * x = from the RED wall (0) to the BLUE wall (144), y = from the AUDIENCE wall (0) to the far wall (144),
 * heading in degrees counter-clockwise from +x (0 = facing the blue wall, 90 = facing the far wall).
 *
 * Auto3/Auto4 work in "our side" numbers (X from OUR alliance wall, Y from the wall behind our near
 * CELL). For red that is the same as field inches; blue is the field turned half a turn.
 *
 * Numbers marked GUESS are not from the manual and can be changed per run (see Scenario).
 */
public final class Field {
    public static final double SIZE = 144, MID = 72;

    // The 4 CELLS (2 per HIVE), SDK tag cluster order: tags 30 + 4 * index ... + 3.
    public static final String[] CELL_NAMES = {"RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING"};
    /** CELL middles, field inches (from Auto4's NEAR_CELL_XY / FAR_CELL_XY, mirrored for blue). */
    public static final double[][] CELL_XY = {{59, 82}, {59, 62}, {85, 62}, {85, 82}};
    /** Tag centers along a CELL's row, inches from its middle (SDK BIOBUZZ tag library). */
    public static final double[] TAG_PLACE = {-6.5, -2.75, 2.75, 6.5};
    public static final double TAG_SIZE = 3.25;
    public static final double HIVE_PIVOT_Y = 72;

    // The shared HIVE frame, 49.46 x 38.95 in. at the field middle; its 4 legs are obstacles.
    public static final double FRAME_HALF_X = 49.46 / 2, FRAME_HALF_Y = 38.95 / 2;
    public static final double LEG_HALF = 1.0;  // GUESS: 2 in. square legs at the frame corners

    /** Start spots, "our side" {X, Y, heading}: where the drive team puts the robot (front on the wall). */
    public static final double[] NEAR_START = {60, 8.5, 270};
    public static final double[] FAR_START = {62, 144 - 8.5, 90};

    /** Our GARDEN POLLEN, "our side" X positions along the near wall (2.8 in. balls against the wall). GUESS. */
    public static final double[] GARDEN_X = {3.5, 7.0, 10.5, 14.0};
    public static final double GARDEN_Y = 1.4;

    private Field() {
    }

    public static double[][] legs() {
        return new double[][] {
                {MID - FRAME_HALF_X, MID - FRAME_HALF_Y}, {MID + FRAME_HALF_X, MID - FRAME_HALF_Y},
                {MID - FRAME_HALF_X, MID + FRAME_HALF_Y}, {MID + FRAME_HALF_X, MID + FRAME_HALF_Y}};
    }

    /** "Our side" X, Y, heading to field. */
    public static double[] toField(boolean blue, double X, double Y, double H) {
        return blue ? new double[] {SIZE - X, SIZE - Y, H + 180} : new double[] {X, Y, H};
    }

    /** Field x, y, heading to "our side". */
    public static double[] toOurs(boolean blue, double x, double y, double h) {
        return blue ? new double[] {SIZE - x, SIZE - y, h - 180} : new double[] {x, y, h};
    }

    /** Our near CELL: red RED AUDIENCE, blue BLUE SCORING (as Auto4). */
    public static int nearCell(boolean blue) {
        return blue ? 3 : 1;
    }

    public static int farCell(boolean blue) {
        return blue ? 2 : 0;
    }

    /** 0 = red HIVE, 1 = blue HIVE. */
    public static int hiveOf(int cell) {
        return cell <= 1 ? 0 : 1;
    }

    /** The CELL of this HIVE on the audience side (y 62). */
    public static boolean isAudienceCell(int cell) {
        return cell == 1 || cell == 2;
    }

    public static double wrapDeg(double d) {
        while (d > 180) d -= 360;
        while (d <= -180) d += 360;
        return d;
    }
}
