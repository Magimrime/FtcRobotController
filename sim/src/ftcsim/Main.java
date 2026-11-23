package ftcsim;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * BIOBUZZ AUTO simulator.
 *   run    [key=value ...]               one match, detailed log + picture
 *   matrix [suite=all|auto4|auto3|stress|quick] [seeds=3] [threads=N] [key=value ...]
 * Results go to sim/results/.
 */
public class Main {

    public static void main(String[] args) throws Exception {
        Locale.setDefault(Locale.US);
        if (args.length == 0 || args[0].equals("help") || args[0].equals("-h")) {
            usage();
            return;
        }
        String cmd = args[0];
        List<String> rest = new ArrayList<>(Arrays.asList(args).subList(1, args.length));
        Map<String, String> kv = Scenario.parse(rest);
        Path out = Paths.get(kv.containsKey("out") ? kv.remove("out") : "sim/results");
        if (cmd.equals("run")) {
            Scenario s = Scenario.of(kv);
            Runner.Result r = Runner.run(s);
            Path dir = out.resolve("single");
            Runner.write(r, dir);
            Path page = Viewer.write(java.util.Collections.singletonList(r), dir, s.name + ".html", s.name);
            System.out.println(s.name);
            System.out.println("  " + r.oneLine());
            for (World.Volley v : r.volleys) System.out.println("  " + v.describe());
            System.out.println("  files: " + dir.resolve(s.name + ".log") + " (.csv, .svg)");
            if (page != null) System.out.println("  replay: " + page.toAbsolutePath() + "  (double-click to open)");
            for (String e : r.events) {
                if (!e.contains("  doing: ")) System.out.println("    " + e);
            }
            System.exit(r.pass ? 0 : 1);
        } else if (cmd.equals("matrix")) {
            String suite = kv.containsKey("suite") ? kv.remove("suite") : "all";
            int seeds = Integer.parseInt(kv.containsKey("seeds") ? kv.remove("seeds") : "3");
            int threads = Integer.parseInt(kv.containsKey("threads") ? kv.remove("threads")
                    : String.valueOf(Math.max(1, Runtime.getRuntime().availableProcessors() - 1)));
            List<Map<String, String>> list = suite(suite, seeds);
            for (Map<String, String> m : list) m.putAll(kv);
            String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss").format(new Date());
            Path dir = out.resolve(suite + "-" + stamp);
            matrix(list, dir, threads, suite, kv);
        } else {
            usage();
        }
    }

    static void usage() {
        System.out.println("BIOBUZZ AUTO simulator: runs the real TeamCode OpMode against a simulated robot and field.\n"
                + "  run    [key=value ...]        one match. e.g. run opmode=Auto4 alliance=blue start=far up=far noise=harsh seed=4\n"
                + "  matrix [suite=all|auto4|auto3|stress|quick] [seeds=3] [threads=N] [key=value ...]\n"
                + "Settings: see sim/README.md (any Scenario field, e.g. strafeEff=0.8 camDropRate=0.2 partnerTip=5).");
    }

    // ================================================================ suites

    static Map<String, String> sc(String... kv) {
        Map<String, String> m = new LinkedHashMap<>();
        for (int i = 0; i + 1 < kv.length; i += 2) m.put(kv[i], kv[i + 1]);
        return m;
    }

    static List<Map<String, String>> suite(String name, int seeds) {
        List<Map<String, String>> out = new ArrayList<>();
        boolean all = name.equals("all");
        // Auto4 (Limelight) and Auto5 (webcam) share the picks and the brain, so they get the same matches
        for (String op : new String[] {"Auto4", "Auto5"}) {
        boolean mine = all || name.equals(op.toLowerCase(Locale.US));
        if (mine || name.equals("quick")) {
            for (String alliance : new String[] {"red", "blue"}) {
                for (String start : new String[] {"near", "far"}) {
                    for (String up : new String[] {"near", "far"}) {
                        out.add(sc("opmode", op, "alliance", alliance, "start", start, "up", up, "noise", "none", "seed", "1"));
                        if (name.equals("quick")) {
                            out.add(sc("opmode", op, "alliance", alliance, "start", start, "up", up, "noise", "typical", "seed", "1"));
                            continue;
                        }
                        for (String noise : new String[] {"typical", "harsh"}) {
                            for (int seed = 1; seed <= seeds; seed++) {
                                out.add(sc("opmode", op, "alliance", alliance, "start", start, "up", up, "noise", noise, "seed", String.valueOf(seed)));
                            }
                        }
                    }
                }
            }
        }
        if (mine || name.equals("stress")) {
            // the driver's X pick (right and wrong), no noise: the logic alone
            out.add(sc("opmode", op, "noise", "none", "up", "near", "pick", "near", "name", op + "-pick-near-right"));
            out.add(sc("opmode", op, "noise", "none", "up", "far", "pick", "far", "name", op + "-pick-far-right"));
            out.add(sc("opmode", op, "noise", "none", "up", "near", "pick", "far", "name", op + "-pick-far-WRONG"));
            out.add(sc("opmode", op, "noise", "none", "up", "far", "pick", "near", "name", op + "-pick-near-WRONG"));
        }
        if (mine || name.equals("stress")) {
            // ONE problem at a time on a perfect robot (red, near start, near CELL up), so each row shows what that problem does
            String[][] stress = {
                    {"no-camera", "camera", "false"},
                    {"camera-no-pictures", "camNoFrames", "true"},
                    {"camera-dropouts", "camDropRate", "0.5", "camDropLen", "1.5"},
                    {"camera-misreads", "camMisread", "0.15"},
                    {"camera-pitch-5deg-off", "camPitch", "20"},
                    {"camera-height-2in-off", "camHeight", "10"},
                    {"upfar-tags-tilt", "up", "far"},
                    {"upfar-tags-dont-tilt", "up", "far", "tagTilt", "0"},
                    {"partner-tips-before-first-look", "partnerTip", "0"},
                    {"partner-tips-before-first-look-tags-dont-tilt", "partnerTip", "0", "tagTilt", "0"},
                    {"partner-tips-at-12s", "partnerTip", "12"},
                    {"needs-5-balls-to-tip", "tipBalls", "5"},
                    {"knock-at-8s", "bumpAt", "8", "bumpX", "4", "bumpY", "3", "bumpH", "10"},
                    {"start-2deg-crooked", "startErrH", "2"},
                    {"start-3in-4deg-off", "startErrX", "3", "startErrH", "4"},
                    {"low-battery", "batteryV0", "11.4", "batteryR", "0.1"},
                    {"strafe-eff-0.84", "strafeEff", "0.84"},
                    {"wheel-slip", "kSlip", "0.4", "aMaxF", "170", "aMaxS", "120", "alphaMax", "14"},
                    {"imu-drift-0.1deg-per-s", "imuBias", "0.1"},
                    {"imu-freezes-at-6s", "imuFreezeAt", "6"},
                    {"dead-encoder-FR", "deadEncoder", "1"},
                    {"slow-siphon-2s-per-turn", "siphonSecPerTurn", "2.0"},
                    {"biobuzz-config", "config", "TeamCode/hardware-config/biobuzz.xml"},
            };
            for (String[] st : stress) {
                Map<String, String> m = sc("opmode", op, "noise", "none", "seed", "1", "name", op + "-stress-" + st[0]);
                for (int i = 1; i + 1 < st.length; i += 2) m.put(st[i], st[i + 1]);
                out.add(m);
            }
        }
        if (mine || name.equals("misses")) {
            // balls that don't go in, and a HIVE that doesn't tip; perfect robot otherwise (red)
            String[][] cases = {
                    {"near-2-of-4-miss-in-volley-1", "missBalls", "1,2"},
                    {"near-all-4-miss-in-volley-1", "missBalls", "1,2,3,4"},
                    {"near-2-of-4-miss-in-volley-2", "missBalls", "5,6"},
                    {"near-hive-does-not-tip-volley-1", "noTip", "1"},
                    {"near-hive-does-not-tip-volley-2", "noTip", "2"},
                    {"near-hive-never-tips", "noTip", "1,2,3,4"},
                    {"near-hive-does-not-tip-volley-1-no-camera", "noTip", "1", "camera", "false"},
                    {"near-hive-does-not-tip-volley-1-tags-dont-tilt", "noTip", "1", "tagTilt", "0"},
                    {"near-2-of-4-miss-in-volley-1-no-camera", "missBalls", "1,2", "camera", "false"},
                    {"far-hive-does-not-tip-volley-1", "start", "far", "up", "far", "noTip", "1"},
                    {"far-2-of-4-miss-in-volley-1", "start", "far", "up", "far", "missBalls", "1,2"},
                    {"far-hive-does-not-tip-volley-1-no-camera", "start", "far", "up", "far", "noTip", "1", "camera", "false"},
                    // the intake fails to pick up GARDEN POLLEN (ball 1 = nearest our alliance wall)
                    {"near-intake-misses-1-GARDEN-ball", "missPickups", "1"},
                    {"near-intake-misses-2-GARDEN-balls", "missPickups", "1,2"},
                    {"near-intake-misses-all-GARDEN-balls", "missPickups", "1,2,3,4"},
                    {"near-only-2-balls-in-GARDEN", "gardenCount", "2"},
                    {"near-GARDEN-empty", "gardenCount", "0"},
                    {"near-intake-misses-1-and-1-bounces-out", "missPickups", "1", "missBalls", "5"},
                    {"far-intake-misses-2-GARDEN-balls", "start", "far", "up", "far", "missPickups", "1,2"},
            };
            for (String[] c : cases) {
                Map<String, String> m = sc("opmode", op, "noise", "none", "seed", "1", "name", op + "-misses-" + c[0]);
                for (int i = 1; i + 1 < c.length; i += 2) m.put(c[i], c[i + 1]);
                out.add(m);
            }
            for (int seed = 1; seed <= seeds; seed++) {
                out.add(sc("opmode", op, "noise", "none", "intakeMiss", "0.25", "seed", String.valueOf(seed),
                        "name", op + "-misses-random-25pct-intake-misses-s" + seed));
            }
            for (String rate : new String[] {"0.3", "0.5"}) {
                for (int seed = 1; seed <= seeds; seed++) {
                    out.add(sc("opmode", op, "noise", "none", "shotMiss", rate, "seed", String.valueOf(seed),
                            "name", op + "-misses-random-" + Math.round(Double.parseDouble(rate) * 100) + "pct-bounce-out-s" + seed));
                }
            }
        }
        }  // for each OpMode
        if (all || name.equals("auto3")) {
            for (String alliance : new String[] {"red", "blue"}) {
                for (String up : new String[] {"near", "far"}) {
                    for (String path : new String[] {"green", "red"}) {
                        out.add(sc("opmode", "Auto3", "alliance", alliance, "up", up, "path", path, "noise", "none", "seed", "1"));
                        for (int seed = 1; seed <= seeds; seed++) {
                            out.add(sc("opmode", "Auto3", "alliance", alliance, "up", up, "path", path, "noise", "typical", "seed", String.valueOf(seed)));
                        }
                    }
                }
            }
        }
        if (out.isEmpty()) throw new IllegalArgumentException("unknown suite " + name);
        return out;
    }

    // ================================================================ matrix + report

    static void matrix(List<Map<String, String>> list, Path dir, int threads, String suite, Map<String, String> common) throws Exception {
        Path runs = dir.resolve("runs");
        Files.createDirectories(runs);
        System.out.println("Running " + list.size() + " matches on " + threads + " threads -> " + dir);
        ExecutorService pool = Executors.newFixedThreadPool(threads);
        List<Future<Runner.Result>> futures = new ArrayList<>();
        for (Map<String, String> m : list) {
            futures.add(pool.submit(() -> {
                Runner.Result r = Runner.run(Scenario.of(m));
                Runner.write(r, runs);
                return r;
            }));
        }
        List<Runner.Result> results = new ArrayList<>();
        for (Future<Runner.Result> f : futures) {
            Runner.Result r = f.get();
            results.add(r);
            System.out.printf(Locale.US, "  %-48s %s%n", r.s.name, r.oneLine());
        }
        pool.shutdown();
        report(results, dir, suite, common);
        Path page = Viewer.write(results, dir, "viewer.html", suite + " · " + dir.getFileName());
        if (page != null) System.out.println("Replay: " + page.toAbsolutePath() + "  (double-click to open in a browser)");
    }

    /** Groups runs that differ only by seed. */
    static String group(Runner.Result r) {
        String n = r.s.name;
        return n.replaceAll("-s\\d+$", "");
    }

    static void report(List<Runner.Result> results, Path dir, String suite, Map<String, String> common) throws IOException {
        List<String> md = new ArrayList<>();
        int pass = 0;
        for (Runner.Result r : results) if (r.pass) pass++;
        md.add("# BIOBUZZ AUTO simulator: " + suite);
        md.add("");
        md.add(String.format(Locale.US, "%d matches, **%d PASS, %d FAIL**. %s", results.size(), pass, results.size() - pass,
                common.isEmpty() ? "" : "Settings for every run: " + common));
        md.add("");
        md.add("PASS = no crash, no midline crossing (G402), no HIVE leg contact, no volley at a DOWN CELL, and at least one TIP. "
                + "Points = 20 per TIP + LEAVE 3 + PARK 5 (the LOADING ZONE is a guess).");
        md.add("");
        md.add("## By scenario");
        md.add("");
        md.add("| Scenario | runs | pass | points (min / avg / max) | TIPS avg | LEAVE | PARK | odometry error max (in.) | closest to midline / HIVE leg (in.) | why it failed |");
        md.add("|---|---|---|---|---|---|---|---|---|---|");
        Map<String, List<Runner.Result>> groups = new LinkedHashMap<>();
        for (Runner.Result r : results) groups.computeIfAbsent(group(r), k -> new ArrayList<>()).add(r);
        for (Map.Entry<String, List<Runner.Result>> g : groups.entrySet()) {
            List<Runner.Result> rs = g.getValue();
            int p = 0, lv = 0, pk = 0, min = 999, max = -1;
            double sum = 0, tips = 0, err = 0, mid = 1e9, leg = 1e9;
            Map<String, Integer> why = new LinkedHashMap<>();
            for (Runner.Result r : rs) {
                mid = Math.min(mid, r.midClearMin);
                leg = Math.min(leg, r.legClearMin);
                if (r.pass) p++;
                if (r.leave) lv++;
                if (r.park) pk++;
                min = Math.min(min, r.points);
                max = Math.max(max, r.points);
                sum += r.points;
                tips += r.tips;
                err = Math.max(err, r.odoErrMax);
                for (String reason : r.failReasons) {
                    String key = reason.replaceAll(",? ?up to [0-9.]+ in\\.", "").replaceAll(" \\(\\d+ times?, [0-9.]+ in\\. deep\\)", "")
                            .replaceAll("^\\d+ volley\\(s\\)", "volley").replaceAll("\\|.*", "").trim();
                    if (key.length() > 90) key = key.substring(0, 90) + "...";
                    why.merge(key, 1, Integer::sum);
                }
            }
            StringBuilder w = new StringBuilder();
            for (Map.Entry<String, Integer> e : why.entrySet()) {
                w.append(w.length() > 0 ? "; " : "").append(e.getKey()).append(rs.size() > 1 ? " (" + e.getValue() + ")" : "");
            }
            md.add(String.format(Locale.US, "| %s | %d | %s | %d / %.0f / %d | %.1f | %d/%d | %d/%d | %.1f | %s / %s | %s |", g.getKey(), rs.size(),
                    p == rs.size() ? "**" + p + "**" : String.valueOf(p), min, sum / rs.size(), max, tips / rs.size(), lv, rs.size(), pk, rs.size(), err,
                    mid > 1e8 ? "-" : String.format(Locale.US, "%.1f", mid), leg > 1e8 ? "-" : String.format(Locale.US, "%.1f", leg),
                    w.toString().replace("|", "/")));
        }
        md.add("");
        md.add("## Every run");
        md.add("");
        md.add("| Run | result | points | TIPS at (s) | volleys | done at (s) | end pose X, Y, heading | odometry error max | notes |");
        md.add("|---|---|---|---|---|---|---|---|---|");
        for (Runner.Result r : results) {
            md.add(String.format(Locale.US, "| [%s](runs/%s.svg) ([log](runs/%s.log)) | %s | %d | %s | %d | %s | %.0f, %.0f, %.0f | %.1f | %s |",
                    r.s.name, r.s.name, r.s.name, r.verdict(), r.points, r.tipTimes.isEmpty() ? "-" : r.tipTimes, r.volleyCount,
                    r.doneAt < 0 ? "busy" : String.format(Locale.US, "%.1f", r.doneAt),
                    r.endPose == null ? 0 : r.endPose[0], r.endPose == null ? 0 : r.endPose[1], r.endPose == null ? 0 : r.endPose[2],
                    r.odoErrMax, (r.pass ? String.join("; ", r.notes) : String.join("; ", r.failReasons)).replace("|", "/")));
        }
        Files.write(dir.resolve("summary.md"), md, StandardCharsets.UTF_8);

        List<String> csv = new ArrayList<>();
        csv.add("name,opmode,alliance,start,up,pick,noise,seed,result,points,tips,tip_times,leave,park,volleys,down_volleys,midline_fouls,leg_hits,done_at,odo_err_max,odo_err_mean,odo_err_end,end_x,end_y,end_h,reasons");
        for (Runner.Result r : results) {
            Scenario s = r.s;
            csv.add(String.format(Locale.US, "%s,%s,%s,%s,%s,%s,%s,%d,%s,%d,%d,\"%s\",%b,%b,%d,%d,%d,%d,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.0f,\"%s\"",
                    s.name, s.opmode, s.alliance, s.start, s.up, s.pick, s.noise, s.seed, r.verdict(), r.points, r.tips, r.tipTimes,
                    r.leave, r.park, r.volleyCount, r.downVolleys, r.midlineFouls, r.legHits, r.doneAt, r.odoErrMax, r.odoErrMean, r.odoErrEnd,
                    r.endPose == null ? 0 : r.endPose[0], r.endPose == null ? 0 : r.endPose[1], r.endPose == null ? 0 : r.endPose[2],
                    String.join("; ", r.pass ? r.notes : r.failReasons).replace("\"", "'")));
        }
        Files.write(dir.resolve("summary.csv"), csv, StandardCharsets.UTF_8);
        System.out.printf(Locale.US, "%n%d matches: %d PASS, %d FAIL.  Report: %s%n", results.size(), pass, results.size() - pass, dir.resolve("summary.md"));
    }
}
