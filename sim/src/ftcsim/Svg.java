package ftcsim;

import java.util.Locale;

/** A top-down picture of one run, as the audience sees the field: true path, the robot's own odometry, volleys, end pose. */
public class Svg {
    static final double S = 4, M = 30;  // pixels per inch, margin

    static double px(double x) {
        return M + x * S;
    }

    static double py(double y) {
        return M + (144 - y) * S;
    }

    static String f(double v) {
        return String.format(Locale.US, "%.1f", v);
    }

    public static String draw(Runner.Result r) {
        Scenario s = r.s;
        boolean blue = s.blue();
        StringBuilder b = new StringBuilder();
        double W = 144 * S + 2 * M, H = 144 * S + 2 * M + 70;
        b.append("<svg xmlns='http://www.w3.org/2000/svg' width='").append(f(W)).append("' height='").append(f(H))
                .append("' viewBox='0 0 ").append(f(W)).append(' ').append(f(H)).append("' font-family='Segoe UI, Arial, sans-serif'>\n");
        b.append("<rect width='100%' height='100%' fill='#ffffff'/>\n");
        // halves
        b.append(rect(0, 0, 72, 144, "#c62828", 0.06, "none"));
        b.append(rect(72, 0, 144, 144, "#1565c0", 0.06, "none"));
        // tiles
        for (int i = 1; i < 6; i++) {
            b.append(line(i * 24, 0, i * 24, 144, "#ddd", 1, ""));
            b.append(line(0, i * 24, 144, i * 24, "#ddd", 1, ""));
        }
        b.append(line(72, 0, 72, 144, "#555", 1.5, "6,4"));
        b.append("<rect x='").append(f(px(0))).append("' y='").append(f(py(144))).append("' width='").append(f(144 * S))
                .append("' height='").append(f(144 * S)).append("' fill='none' stroke='#222' stroke-width='3'/>\n");
        b.append(text(px(72), py(-4) + 14, "AUDIENCE", 12, "#444", "middle"));
        b.append(text(px(-2) - 10, py(72), "RED", 12, "#c62828", "middle"));
        b.append(text(px(146) + 10, py(72), "BLUE", 12, "#1565c0", "middle"));
        // HIVE frame and legs
        b.append(rect(72 - Field.FRAME_HALF_X, 72 - Field.FRAME_HALF_Y, 72 + Field.FRAME_HALF_X, 72 + Field.FRAME_HALF_Y, "none", 0, "#333\" stroke-dasharray=\"5,4"));
        for (double[] leg : s.hiveLegs ? Field.legs() : new double[0][]) {
            b.append(rect(leg[0] - s.legHalf, leg[1] - s.legHalf, leg[0] + s.legHalf, leg[1] + s.legHalf, "#333", 1, "none"));
        }
        // CELLs
        for (int c = 0; c < 4; c++) {
            double[] xy = Field.CELL_XY[c];
            boolean red = c <= 1;
            b.append(rect(xy[0] - s.cellOpenHalfX, xy[1] - s.cellOpenHalfY, xy[0] + s.cellOpenHalfX, xy[1] + s.cellOpenHalfY,
                    red ? "#c62828" : "#1565c0", 0.25, red ? "#c62828" : "#1565c0"));
            String tag = c == Field.nearCell(blue) ? "our near" : c == Field.farCell(blue) ? "our far" : "";
            b.append(text(px(xy[0]), py(xy[1]) + 4, tag, 10, "#000", "middle"));
        }
        // LOADING ZONE guess and GARDEN
        double[] a = Field.toField(blue, s.lzX0, s.lzY0, 0), c2 = Field.toField(blue, s.lzX1, s.lzY1, 0);
        b.append(rect(Math.min(a[0], c2[0]), Math.min(a[1], c2[1]), Math.max(a[0], c2[0]), Math.max(a[1], c2[1]), "#f9a825", 0.15, "#f9a825\" stroke-dasharray=\"4,3"));
        double[] lzc = Field.toField(blue, (s.lzX0 + s.lzX1) / 2, (s.lzY0 + s.lzY1) / 2, 0);
        b.append(text(px(lzc[0]), py(lzc[1]), "LOADING ZONE?", 9, "#8d6e00", "middle"));
        for (double gx : Field.GARDEN_X) {
            double[] g = Field.toField(blue, gx, Field.GARDEN_Y, 0);
            b.append("<circle cx='").append(f(px(g[0]))).append("' cy='").append(f(py(g[1]))).append("' r='").append(f(1.4 * S))
                    .append("' fill='#fdd835' stroke='#9e8c00'/>\n");
        }
        // paths
        StringBuilder truth = new StringBuilder(), odo = new StringBuilder();
        for (double[] row : r.trace) {
            double[] t = Field.toField(blue, row[1], row[2], 0);
            truth.append(truth.length() == 0 ? "M" : "L").append(f(px(t[0]))).append(',').append(f(py(t[1])));
            if (!Double.isNaN(row[4])) {
                double[] o = Field.toField(blue, row[4], row[5], 0);
                odo.append(odo.length() == 0 ? "M" : "L").append(f(px(o[0]))).append(',').append(f(py(o[1])));
            }
        }
        if (odo.length() > 0) {
            b.append("<path d='").append(odo).append("' fill='none' stroke='#8e24aa' stroke-width='1.5' stroke-dasharray='4,3'/>\n");
        }
        b.append("<path d='").append(truth).append("' fill='none' stroke='#2e7d32' stroke-width='2.2'/>\n");
        // volleys
        int i = 1;
        for (World.Volley v : r.volleys) {
            double[] p = Field.toField(blue, v.X, v.Y, v.H);
            double hr = Math.toRadians(p[2] + 180);
            b.append(line(p[0], p[1], p[0] + 53 * Math.cos(hr), p[1] + 53 * Math.sin(hr), v.targetWasUp ? "#ef6c00" : "#d50000", 1.5, "2,2"));
            b.append("<circle cx='").append(f(px(p[0]))).append("' cy='").append(f(py(p[1]))).append("' r='7' fill='")
                    .append(v.targetWasUp ? "#ef6c00" : "#d50000").append("'/>\n");
            b.append(text(px(p[0]), py(p[1]) + 4, String.valueOf(i++), 10, "#fff", "middle"));
        }
        // start and end robot
        if (!r.trace.isEmpty()) {
            double[] st = r.trace.get(0);
            b.append(robot(Field.toField(blue, st[1], st[2], st[3]), s, "#9e9e9e"));
        }
        if (r.endPose != null) {
            b.append(robot(Field.toField(blue, r.endPose[0], r.endPose[1], r.endPose[2]), s, r.pass ? "#2e7d32" : "#c62828"));
        }
        // caption
        double y0 = 144 * S + 2 * M + 4;
        b.append(text(M, y0 + 14, s.name, 14, "#000", "start"));
        b.append(text(M, y0 + 32, r.verdict() + "  " + r.points + " pts  (" + r.tips + " TIPS" + (r.leave ? ", LEAVE" : "") + (r.park ? ", PARK" : "") + ")   "
                + (r.pass ? String.join("; ", r.notes) : String.join("; ", r.failReasons)), 12, r.pass ? "#2e7d32" : "#c62828", "start"));
        b.append(text(M, y0 + 50, "green = true path   purple dashed = where the robot's odometry thought it was   circles = volleys (orange at an up CELL, red at a DOWN one)   grey = start, box = end", 10, "#444", "start"));
        b.append("</svg>\n");
        return b.toString();
    }

    static String robot(double[] p, Scenario s, String color) {
        double h = Math.toRadians(p[2]), c = Math.cos(h), sn = Math.sin(h);
        double[][] k = {{s.halfLen, s.halfWid}, {s.halfLen, -s.halfWid}, {-s.halfLen, -s.halfWid}, {-s.halfLen, s.halfWid}};
        StringBuilder d = new StringBuilder();
        for (double[] q : k) {
            double x = p[0] + q[0] * c + q[1] * sn, y = p[1] + q[0] * sn - q[1] * c;
            d.append(d.length() == 0 ? "M" : "L").append(f(px(x))).append(',').append(f(py(y)));
        }
        double fx = p[0] + s.halfLen * c, fy = p[1] + s.halfLen * sn;
        return "<path d='" + d + "Z' fill='" + color + "' fill-opacity='0.15' stroke='" + color + "' stroke-width='2'/>\n"
                + line(p[0], p[1], fx, fy, color, 2, "");
    }

    static String rect(double x0, double y0, double x1, double y1, String fill, double op, String stroke) {
        return "<rect x='" + f(px(x0)) + "' y='" + f(py(y1)) + "' width='" + f((x1 - x0) * S) + "' height='" + f((y1 - y0) * S)
                + "' fill='" + fill + "' fill-opacity='" + op + "' stroke=\"" + stroke + "\"/>\n";
    }

    static String line(double x0, double y0, double x1, double y1, String color, double wdt, String dash) {
        return "<line x1='" + f(px(x0)) + "' y1='" + f(py(y0)) + "' x2='" + f(px(x1)) + "' y2='" + f(py(y1)) + "' stroke='" + color
                + "' stroke-width='" + wdt + "'" + (dash.isEmpty() ? "" : " stroke-dasharray='" + dash + "'") + "/>\n";
    }

    static String text(double x, double y, String t, int size, String color, String anchor) {
        return "<text x='" + f(x) + "' y='" + f(y) + "' font-size='" + size + "' fill='" + color + "' text-anchor='" + anchor + "'>"
                + t.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;") + "</text>\n";
    }
}
