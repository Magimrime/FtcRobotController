package ftcsim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Writes the replay page: sim/viewer/viewer.html with every run's data put inside it, so it opens
 * by double-click with no server. The same data also goes to viewer-data.json next to it (the page
 * loads that file when it has no data inside, e.g. when published).
 */
public class Viewer {
    static final String TEMPLATE = "sim/viewer/viewer.html";
    static final String MARK = "<!--SIM_DATA-->";

    public static Path write(List<Runner.Result> results, Path dir, String fileName, String title) throws IOException {
        String json = json(results, title);
        Files.createDirectories(dir);
        Files.write(dir.resolve("viewer-data.json"), json.getBytes(StandardCharsets.UTF_8));
        Path template = Paths.get(TEMPLATE);
        if (!Files.exists(template)) {
            System.out.println("(no " + TEMPLATE + ": replay page not written)");
            return null;
        }
        String page = new String(Files.readAllBytes(template), StandardCharsets.UTF_8);
        String data = "<script>window.SIM_DATA = " + json.replace("</", "<\\/") + ";</script>";
        // the template is a page body (as published); a file opened from disk needs the document around it
        String html = "<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
                + "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n"
                + "<style>body{margin:0}[hidden]{display:none!important}</style>\n</head>\n<body>\n"
                + page.replace(MARK, data) + "\n</body>\n</html>\n";
        Path out = dir.resolve(fileName);
        Files.write(out, html.getBytes(StandardCharsets.UTF_8));
        return out;
    }

    static String json(List<Runner.Result> results, String title) {
        StringBuilder b = new StringBuilder(1 << 20);
        b.append("{\"title\":").append(q(title)).append(",\"runs\":[");
        boolean first = true;
        for (Runner.Result r : results) {
            if (!first) b.append(",\n");
            first = false;
            run(b, r);
        }
        return b.append("]}").toString();
    }

    static void run(StringBuilder b, Runner.Result r) {
        Scenario s = r.s;
        boolean blue = s.blue();
        b.append('{');
        kv(b, "name", s.name);
        kv(b, "group", Main.group(r));
        kv(b, "opmode", s.opmode);
        kv(b, "alliance", s.alliance);
        kv(b, "start", s.opmode.equals("Auto3") ? "near" : s.start);
        kv(b, "up", s.up);
        kv(b, "pick", s.pick);
        kv(b, "path", s.path);
        kv(b, "noise", s.noise);
        b.append("\"seed\":").append(s.seed).append(',');
        b.append("\"pass\":").append(r.pass).append(',');
        b.append("\"points\":").append(r.points).append(',');
        b.append("\"tips\":").append(r.tips).append(',');
        b.append("\"leave\":").append(r.leave).append(',');
        b.append("\"park\":").append(r.park).append(',');
        num(b, "odoErrMax", r.odoErrMax);
        num(b, "doneAt", r.doneAt);
        num(b, "midClear", r.midClearMin > 1e8 ? 99 : r.midClearMin);
        num(b, "legClear", r.legClearMin > 1e8 ? 99 : r.legClearMin);
        list(b, "fail", r.failReasons);
        list(b, "notes", r.notes);
        // what makes this robot / field different from the code's assumptions
        Map<String, String> setup = new LinkedHashMap<>();
        setup.put("strafe efficiency (code 0.90)", f2(s.strafeEff));
        setup.put("wheel slip", f2(s.kSlip));
        setup.put("start off by", String.format(Locale.US, "%.1f in., %.1f deg", s.startErrX, s.startErrH));
        setup.put("IMU drift", String.format(Locale.US, "%.3f deg/s", s.imuBias));
        setup.put("battery", String.format(Locale.US, "%.1f V", s.batteryV0));
        setup.put("camera pitch (code 15)", String.format(Locale.US, "%.1f deg", s.camPitch));
        setup.put("siphon s/turn (code 1.5)", f2(s.siphonSecPerTurn));
        setup.put("POLLEN to tip", String.valueOf(s.tipBalls));
        if (s.partnerTip >= 0) setup.put("partner tips our HIVE at", s.partnerTip + " s");
        if (s.bumpAt >= 0) setup.put("knock at", s.bumpAt + " s");
        if (!s.camera) setup.put("camera", "none");
        if (s.camNoFrames) setup.put("camera", "no pictures");
        b.append("\"setup\":[");
        boolean f1 = true;
        for (Map.Entry<String, String> e : setup.entrySet()) {
            if (!f1) b.append(',');
            f1 = false;
            b.append('[').append(q(e.getKey())).append(',').append(q(e.getValue())).append(']');
        }
        b.append("],");
        // field pieces, field inches as the audience sees it
        b.append("\"robot\":[").append(f1(s.halfLen)).append(',').append(f1(s.halfWid)).append("],");
        b.append("\"cam\":[").append(f1(s.camBack)).append(',').append(f1(s.camHfov)).append("],");
        b.append("\"cellOpen\":[").append(f1(s.cellOpenHalfX)).append(',').append(f1(s.cellOpenHalfY)).append("],");
        b.append("\"legHalf\":").append(s.hiveLegs ? f1(s.legHalf) : "0").append(',');
        double[] a = Field.toField(blue, s.lzX0, s.lzY0, 0), c = Field.toField(blue, s.lzX1, s.lzY1, 0);
        b.append("\"lz\":[").append(f1(Math.min(a[0], c[0]))).append(',').append(f1(Math.min(a[1], c[1]))).append(',')
                .append(f1(Math.max(a[0], c[0]))).append(',').append(f1(Math.max(a[1], c[1]))).append("],");
        b.append("\"garden\":[");
        for (int i = 0; i < r.garden.size(); i++) {
            double[] g = r.garden.get(i);
            if (i > 0) b.append(',');
            b.append('[').append(f1(g[0])).append(',').append(f1(g[1])).append(',').append(f1(r.gardenTaken[i])).append(',')
                    .append(f1(r.gardenMissed[i])).append(']');
        }
        b.append("],");
        b.append("\"balls\":[");
        for (int i = 0; i < r.balls.size(); i++) {
            double[] x = r.balls.get(i);
            if (i > 0) b.append(',');
            b.append('[').append(f2(x[0])).append(',').append(f2(x[1])).append(',').append(f1(x[2])).append(',').append(f1(x[3]))
                    .append(',').append(f1(x[4])).append(',').append(f1(x[5])).append(',').append((int) x[6]).append(']');
        }
        b.append("],");
        // samples at 10 per second, columnar
        List<String> steps = new ArrayList<>();
        StringBuilder[] col = new StringBuilder[15];
        for (int i = 0; i < col.length; i++) col[i] = new StringBuilder();
        int n = 0;
        for (int i = 0; i < r.trace.size(); i += 2) {
            double[] row = r.trace.get(i);
            double[] t = Field.toField(blue, row[1], row[2], row[3]);
            String sep = n == 0 ? "" : ",";
            col[0].append(sep).append(f2(row[0]));
            col[1].append(sep).append(f1(t[0]));
            col[2].append(sep).append(f1(t[1]));
            col[3].append(sep).append(f1(Field.wrapDeg(t[2])));
            if (Double.isNaN(row[4])) {
                col[4].append(sep).append("null");
                col[5].append(sep).append("null");
                col[6].append(sep).append("null");
            } else {
                double[] o = Field.toField(blue, row[4], row[5], row[6]);
                col[4].append(sep).append(f1(o[0]));
                col[5].append(sep).append(f1(o[1]));
                col[6].append(sep).append(f1(Field.wrapDeg(o[2])));
            }
            col[7].append(sep).append((int) row[8]);
            col[8].append(sep).append(Math.round(row[9]));
            col[9].append(sep).append(f1(row[10]));
            col[10].append(sep).append((int) row[11]);
            col[11].append(sep).append((int) row[12]);
            col[12].append(sep).append((int) row[13]);
            col[13].append(sep).append((int) row[14] | ((int) row[15] << 1) | ((int) row[7] << 2));
            String st = r.traceStep.get(i);
            int idx = steps.indexOf(st);
            if (idx < 0) {
                steps.add(st);
                idx = steps.size() - 1;
            }
            col[14].append(sep).append(idx);
            n++;
        }
        String[] names = {"t", "x", "y", "h", "ox", "oy", "oh", "held", "rpm", "v", "hr", "hb", "tags", "flags", "step"};
        b.append("\"s\":{");
        for (int i = 0; i < names.length; i++) {
            if (i > 0) b.append(',');
            b.append(q(names[i])).append(":[").append(col[i]).append(']');
        }
        b.append("},");
        list(b, "steps", steps);
        b.append("\"events\":[");
        boolean fe = true;
        for (String e : r.events) {
            if (e.contains("  doing: ")) continue;
            String text = e.length() > 8 ? e.substring(8).trim() : e;
            double t;
            try {
                t = Double.parseDouble(e.substring(0, 6).trim());
            } catch (RuntimeException ex) {
                continue;
            }
            if (!fe) b.append(',');
            fe = false;
            b.append('[').append(f2(t)).append(',').append(q(text)).append(']');
        }
        b.append("],");
        list(b, "volleys", describe(r.volleys));
        list(b, "initScreen", r.initScreen);
        list(b, "endScreen", r.finalScreen);
        b.setLength(b.length() - 1);  // trailing comma
        b.append('}');
    }

    static List<String> describe(List<World.Volley> vs) {
        List<String> out = new ArrayList<>();
        for (World.Volley v : vs) out.add(v.describe().trim());
        return out;
    }

    static void kv(StringBuilder b, String k, String v) {
        b.append(q(k)).append(':').append(q(v)).append(',');
    }

    static void num(StringBuilder b, String k, double v) {
        b.append(q(k)).append(':').append(Double.isNaN(v) || Double.isInfinite(v) ? "null" : f2(v)).append(',');
    }

    static void list(StringBuilder b, String k, List<String> v) {
        b.append(q(k)).append(":[");
        for (int i = 0; i < v.size(); i++) {
            if (i > 0) b.append(',');
            b.append(q(v.get(i)));
        }
        b.append("],");
    }

    static String f1(double d) {
        return String.format(Locale.US, "%.1f", d);
    }

    static String f2(double d) {
        return String.format(Locale.US, "%.2f", d);
    }

    static String q(String s) {
        StringBuilder b = new StringBuilder("\"");
        for (char c : s.toCharArray()) {
            switch (c) {
                case '"': b.append("\\\""); break;
                case '\\': b.append("\\\\"); break;
                case '\n': b.append("\\n"); break;
                case '\r': break;
                case '\t': b.append(' '); break;
                default:
                    if (c < 0x20) b.append(' ');
                    else b.append(c);
            }
        }
        return b.append('"').toString();
    }
}
