package ftcsim;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;

/**
 * The physical match: the robot's true motion, its mechanisms, the POLLEN, both HIVEs, and the
 * rule checks. Field inches as the audience sees it (see Field).
 *
 * DRIVE: mecanum. Each wheel's motor wants a rim speed (power x free speed x battery / 12 V x its
 * strength). While the tires grip, the robot follows those speeds with a lag (tauBody) and the
 * encoders count exactly the robot's motion (sideways inflated by 1 / strafeEff, as rollers slip).
 * When the change asked for needs more grip than there is (aMaxF / aMaxS / alphaMax), the robot
 * only accelerates at the grip limit and the wheels spin ahead or skid (kSlip): the encoders count
 * inches the robot never moved. Walls and HIVE legs stop the body; the wheels keep turning.
 */
public class World {
    final SimContext ctx;
    final Scenario s;
    final boolean blue;
    final Random rng;

    // ---- robot truth
    double x, y, hDeg;            // field inches, heading (unwrapped degrees)
    double vf, vs, w;             // robot frame: in/s forward, in/s right, rad/s counter-clockwise
    final double[] wr = new double[4];      // wheel rim speeds, in/s (+ = drives forward)
    final double[] travel = new double[4];  // wheel rim inches turned
    final double[] intent = new double[3];  // the twist the motors ask for (for wall pushing)
    boolean slipping, gripping;
    double volts;

    // ---- mechanisms
    double rpm, flyTicks, siphonPos, nextLaunchAt = 0.2, intakeTicks;
    int held = 4;
    final List<double[]> garden = new ArrayList<>();  // field x, y, taken (0/1)
    final double[] gardenTaken = {-1, -1, -1, -1};    // match second each GARDEN POLLEN was picked up
    final double[] gardenMissed = {-1, -1, -1, -1};   // match second the intake missed it
    /** Every ball shot: launch s, land s, launch x, y, land x, y (field), outcome 0 missed 1 up CELL 2 DOWN CELL 3 late 4 bounced out. */
    public final List<double[]> balls = new ArrayList<>();

    // ---- HIVEs: [0] red, [1] blue. audienceUp = the CELL at y 62 points up.
    final boolean[] audienceUp = new boolean[2];
    final int[] cellCount = new int[4];
    final double[] tipAt = {-1, -1};
    int ourTips, partnerTips, otherTips;
    final List<String> tipTimes = new ArrayList<>();

    // ---- shots
    static class Shot {
        double launchT, landT, x0, y0, lx, ly, rpm;
        boolean targetUpAtLaunch;  // the CELL this ball is heading for was up when it left the robot
        int number;                // 1 = first ball shot in the match
        Volley volley;
    }

    public static class Volley {
        public double t, X, Y, H, rpmSum;  // match time, true pose "our side"
        public int number, balls, intoUp, intoDown, late, missed, bounced, launchedAtDown;
        public boolean noTip;              // the scenario says this volley's balls don't tip the HIVE
        public String aimedAt = "";        // near / far CELL (the one nearest where the balls come down)
        public double aimErrDeg, dist;
        public boolean targetWasUp;
        public double lastLaunch;
        public boolean tipped;

        public String describe() {
            return String.format(Locale.US, "%5.1f s  volley of %d at our %s CELL (%s when shot), aim off %+.1f deg, %.0f in., %.0f RPM:"
                            + " %d in the up CELL, %d into a DOWN CELL%s, %d missed, %d bounced out%s   [true pose X %.1f Y %.1f heading %.0f]",
                    t, balls, aimedAt, targetWasUp ? "UP" : "DOWN", aimErrDeg, dist, rpmSum / Math.max(1, balls),
                    intoUp, intoDown, late > 0 ? " + " + late + " late (launched while it was up, landed after it tipped)" : "",
                    missed, bounced, tipped ? "  -> TIPPED" : noTip && intoUp > 0 ? "  -> HIVE DID NOT TIP (scenario)" : "", X, Y, H);
        }
    }

    final List<Shot> flying = new ArrayList<>();
    public final List<Volley> volleys = new ArrayList<>();
    Volley current;

    // ---- rule checks
    int midlineFouls, legHits;
    double midlineMax, legDepthMax;
    double legClearMin = 1e9, midClearMin = 1e9;  // closest approach in inches (negative = into it)
    boolean overMid, onLeg;
    final List<String> fouls = new ArrayList<>();

    // ---- trace (match only): t, trueX, trueY, trueH, odoX, odoY, odoH ("our side")
    public final List<double[]> trace = new ArrayList<>();
    public final List<String> traceStep = new ArrayList<>();
    double nextTrace = 0;
    double odoErrMax = 0, odoErrSum = 0, odoHeadErrMax = 0, lastWallContact = -1;
    int odoErrN = 0;
    double[] endPose;             // true "our side" pose at the end of AUTO
    double[] endOdo;
    boolean endTouchingWall, endInLoadingZone;
    double endWallGap;
    boolean partnerDone = false, bumpDone = false;

    World(SimContext ctx) {
        this.ctx = ctx;
        this.s = ctx.scenario;
        this.blue = s.blue();
        this.rng = new Random(s.seed * 1000003L + 11);
        volts = s.batteryV0;
        // HIVE start state
        int ourHive = Field.hiveOf(Field.nearCell(blue));
        boolean nearUp = s.up.equals("near");
        audienceUp[ourHive] = Field.isAudienceCell(Field.nearCell(blue)) == nearUp;
        int theirHive = 1 - ourHive;
        audienceUp[theirHive] = s.theirUp.equals("audience") || (s.theirUp.equals("random") && rng.nextBoolean());
        for (int i = 0; i < Math.min(s.gardenCount, Field.GARDEN_X.length); i++) {
            double[] f = Field.toField(blue, Field.GARDEN_X[i], Field.GARDEN_Y, 0);
            garden.add(new double[] {f[0], f[1], 0});  // x, y, state: 0 lying, 1 picked up, 2 the intake missed it
        }
    }

    void placeRobot() {
        double[] spot = s.opmode.equals("Auto3") || s.start.equals("near") ? Field.NEAR_START : Field.FAR_START;
        boolean near = spot == Field.NEAR_START;
        double X = spot[0] + s.startErrX, Y = spot[1] + (near ? s.startErrY : -s.startErrY), H = spot[2] + s.startErrH;
        double[] f = Field.toField(blue, X, Y, H);
        x = f[0];
        y = f[1];
        hDeg = f[2];
        collide(0.001);
        vf = vs = w = 0;
    }

    public double trueHeadingDeg() {
        return hDeg;
    }

    /** True pose, "our side". */
    public double[] ours() {
        double[] o = Field.toOurs(blue, x, y, hDeg);
        o[2] = ((o[2] % 360) + 360) % 360;
        return o;
    }

    // ================================================================ one physics step

    void step(double dt, double t) {
        double mt = t - s.initTime;
        battery(dt, mt);
        drive(dt);
        double h = Math.toRadians(hDeg);
        x += (vf * Math.cos(h) + vs * Math.sin(h)) * dt;
        y += (vf * Math.sin(h) - vs * Math.cos(h)) * dt;
        hDeg += Math.toDegrees(w * dt);
        collide(dt);
        if (gripping) {
            double[] ib = ik(vf, vs, w);
            System.arraycopy(ib, 0, wr, 0, 4);
        }
        for (int i = 0; i < 4; i++) {
            travel[i] += wr[i] * dt;
        }
        if (mt >= 0) {
            rules(mt);
        }
        flywheel(dt);
        siphon(t, dt);
        intake(dt);
        land(t);
        tips(t, mt);
        events(mt);
        ctx.camera.step(t + dt);
        if (ctx.portal != null) ctx.portal.step(t + dt);
        encoders();
        if (mt >= 0 && mt >= nextTrace && mt <= s.matchTime + 0.6) {
            record(mt);
            nextTrace += 0.05;
        }
        if (endPose == null && mt >= s.matchTime + 0.5) {
            endOfAuto();
        }
    }

    // ================================================================ drive

    void battery(double dt, double mt) {
        double amps = 0;
        for (SimMotor m : ctx.wheels) {
            if (m != null) amps += Math.abs(m.usefulPower()) * 4.5;
        }
        if (ctx.intake != null) amps += Math.abs(ctx.intake.usefulPower()) * 2;
        double target = flyTarget();
        amps += Math.abs(target - rpm) > 150 ? 9 : Math.abs(rpm) > 100 ? 2.5 : 0;
        double v = s.batteryV0 - s.batteryDrain * Math.max(0, mt) - s.batteryR * amps;
        volts += (v - volts) * Math.min(1, dt / 0.05);
    }

    void drive(double dt) {
        double[] want = new double[4];
        for (int i = 0; i < 4; i++) {
            SimMotor m = ctx.wheels[i];
            double p = m == null ? 0 : m.usefulPower();
            want[i] = p * s.wheelFreeSpeed * volts / 12.0 * s.wheelGain[i];
        }
        double[] bt = fk(want);
        System.arraycopy(bt, 0, intent, 0, 3);
        double k = Math.min(1, dt / s.tauBody);
        double nf = vf + (bt[0] - vf) * k, ns = vs + (bt[1] - vs) * k, nw = w + (bt[2] - w) * k;
        double n = Math.sqrt(sq((nf - vf) / dt / s.aMaxF) + sq((ns - vs) / dt / s.aMaxS) + sq((nw - w) / dt / s.alphaMax));
        if (n <= 1) {
            vf = nf;
            vs = ns;
            w = nw;
            gripping = true;
            slipping = false;
        } else {
            vf += (nf - vf) / n;
            vs += (ns - vs) / n;
            w += (nw - w) / n;
            double[] ib = ik(vf, vs, w);
            double kw = Math.min(1, dt / s.tauWheelSlip);
            for (int i = 0; i < 4; i++) {
                double target = ib[i] + s.kSlip * (want[i] - ib[i]);
                wr[i] += (target - wr[i]) * kw;
            }
            gripping = false;
            slipping = true;
        }
    }

    /** Wheel rim speeds -> robot twist (forward, right, counter-clockwise rad/s). */
    double[] fk(double[] v) {
        return new double[] {(v[0] + v[1] + v[2] + v[3]) / 4,
                (v[0] - v[1] - v[2] + v[3]) / 4 * s.strafeEff,
                (-v[0] + v[1] - v[2] + v[3]) / (4 * s.wheelK) * s.turnEff};
    }

    /** Robot twist -> the rim speeds that make it without extra slip. */
    double[] ik(double f, double r, double om) {
        double rr = r / s.strafeEff, oo = om / s.turnEff * s.wheelK;
        return new double[] {f + rr - oo, f - rr + oo, f - rr - oo, f + rr + oo};
    }

    static double sq(double a) {
        return a * a;
    }

    double[][] corners() {
        double h = Math.toRadians(hDeg), c = Math.cos(h), sn = Math.sin(h);
        double[][] out = new double[4][];
        int i = 0;
        for (int a = -1; a <= 1; a += 2) {
            for (int b = -1; b <= 1; b += 2) {
                double f = a * s.halfLen, r = b * s.halfWid;
                out[i++] = new double[] {x + f * c + r * sn, y + f * sn - r * c};
            }
        }
        return out;
    }

    /** Walls and HIVE legs push the robot back; the velocity into them is lost; a robot pushed flat on a wall squares up to it. */
    void collide(double dt) {
        boolean legContact = false;
        double legDepth = 0;
        for (int iter = 0; iter < 3; iter++) {
            double h = Math.toRadians(hDeg), c = Math.cos(h), sn = Math.sin(h);
            double vx = vf * c + vs * sn, vy = vf * sn - vs * c;
            double ix = intent[0] * c + intent[1] * sn, iy = intent[0] * sn - intent[1] * c;
            double[][] cs = corners();
            double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
            for (double[] p : cs) {
                minX = Math.min(minX, p[0]);
                maxX = Math.max(maxX, p[0]);
                minY = Math.min(minY, p[1]);
                maxY = Math.max(maxY, p[1]);
            }
            double push = 0;
            boolean hit = false;
            if (minX < 0) { x -= minX; if (vx < 0) vx = 0; push = Math.max(push, -ix); hit = true; }
            if (maxX > Field.SIZE) { x -= maxX - Field.SIZE; if (vx > 0) vx = 0; push = Math.max(push, ix); hit = true; }
            if (minY < 0) { y -= minY; if (vy < 0) vy = 0; push = Math.max(push, -iy); hit = true; }
            if (maxY > Field.SIZE) { y -= maxY - Field.SIZE; if (vy > 0) vy = 0; push = Math.max(push, iy); hit = true; }
            if (hit) {
                lastWallContact = ctx.now();
            }
            if (hit && push > 0.5 && dt > 0) {
                double err = hDeg - 90 * Math.round(hDeg / 90);
                double turn = Math.min(Math.abs(err), Math.toDegrees(s.wallSquare * push * dt));
                hDeg -= Math.signum(err) * turn;
            }
            // HIVE frame legs (GUESS: square posts at the frame corners)
            for (double[] leg : s.hiveLegs ? Field.legs() : new double[0][]) {
                double dx = leg[0] - x, dy = leg[1] - y;
                double f = dx * c + dy * sn, r = dx * sn - dy * c;
                double penF = s.halfLen + s.legHalf - Math.abs(f), penR = s.halfWid + s.legHalf - Math.abs(r);
                if (iter == 0 && ctx.isStarted() && ctx.matchTime() <= s.matchTime) {
                    // closest approach of the robot's outline to a leg (negative = overlapping)
                    double gap = penF > 0 && penR > 0 ? -Math.min(penF, penR) : Math.hypot(Math.max(-penF, 0), Math.max(-penR, 0));
                    legClearMin = Math.min(legClearMin, gap);
                }
                if (penF > 0 && penR > 0) {
                    legContact = true;
                    legDepth = Math.max(legDepth, Math.min(penF, penR));
                    if (penF < penR) {
                        x -= Math.signum(f) * penF * c;
                        y -= Math.signum(f) * penF * sn;
                        double along = vx * c + vy * sn;
                        if (along * Math.signum(f) > 0) { vx -= along * c; vy -= along * sn; }
                    } else {
                        x -= Math.signum(r) * penR * sn;
                        y += Math.signum(r) * penR * c;
                        double side = vx * sn - vy * c;
                        if (side * Math.signum(r) > 0) { vx -= side * sn; vy += side * c; }
                    }
                    hit = true;
                }
            }
            vf = vx * c + vy * sn;
            vs = vx * sn - vy * c;
            if (!hit) break;
        }
        // one contact = touching again within 0.3 s counts as the same one
        if (ctx.isStarted() && legContact && ctx.now() - lastLegContact > 0.3) {
            legHits++;
            fouls.add(String.format(Locale.US, "%.2f s  touched a HIVE frame leg (G417 risk: moving the HIVE)", ctx.matchTime()));
            ctx.event("!! robot touches a HIVE frame leg");
        }
        if (legContact) {
            legDepthMax = Math.max(legDepthMax, legDepth);
            lastLegContact = ctx.now();
        }
    }

    double lastLegContact = -1;

    /** G402: no part of the robot on the other alliance's half during AUTO. */
    void rules(double mt) {
        if (mt > s.matchTime) return;
        double worst = -1e9;
        for (double[] p : corners()) {
            worst = Math.max(worst, blue ? Field.MID - p[0] : p[0] - Field.MID);
        }
        midClearMin = Math.min(midClearMin, -worst);
        if (worst > 0) {
            midlineMax = Math.max(midlineMax, worst);
            if (!overMid) {
                midlineFouls++;
                fouls.add(String.format(Locale.US, "%.2f s  CROSSED THE MIDLINE (G402 MAJOR FOUL)", mt));
                ctx.event("!! robot crosses the midline (G402)");
            }
        }
        overMid = worst > 0;
    }

    // ================================================================ mechanisms

    double flyTarget() {
        SimMotor fm = ctx.flywheel;
        if (fm == null) return 0;
        return fm.velocityControlled() ? fm.usefulVelocityTarget() * 60 / 28 : fm.usefulPower() * 6000;
    }

    void flywheel(double dt) {
        double max = 6000 * volts / 12 * 0.93;
        double target = Math.max(-max, Math.min(max, flyTarget()));
        double accMax = s.flywheelAccel * volts / 12;
        double acc = (target - rpm) / s.flywheelTau;
        if (target == 0) acc = Math.max(-300, Math.min(300, acc));  // coasting down
        acc = Math.max(-2 * accMax, Math.min(accMax, acc));
        rpm += acc * dt;
        flyTicks += rpm / 60 * 28 * dt;
    }

    /** The siphon feeds one ball into the flywheel every quarter turn (one turn = all 4). */
    void siphon(double t, double dt) {
        double p = ctx.siphon == null ? 0 : ctx.siphon.usefulPower();
        siphonPos += p / (s.siphonSecPerTurn * 0.3) * dt;
        while (held > 0 && siphonPos >= nextLaunchAt) {
            launch(t);
            held--;
            nextLaunchAt += 0.25;
        }
    }

    void launch(double t) {
        double mt = t - s.initTime;
        double[] o = ours();
        if (current == null || mt - current.lastLaunch > 1.5) {
            current = new Volley();
            current.t = mt;
            current.X = o[0];
            current.Y = o[1];
            current.H = o[2];
            // which of our CELLs this volley goes at: the one nearest where a ball at this RPM comes down
            // (both CELLs are in a line behind the robot, so the range decides, not the direction)
            double range = s.shotRangeRef * sq(Math.max(rpm, 1) / s.shotRpmRef);
            double dir = Math.toRadians(hDeg + 180 + s.launcherYawErr);
            double lx = x + range * Math.cos(dir), ly = y + range * Math.sin(dir);
            double best = 1e9;
            for (int which = 0; which < 2; which++) {
                int cell = which == 0 ? Field.nearCell(blue) : Field.farCell(blue);
                double[] cxy = Field.CELL_XY[cell];
                double bearing = Math.toDegrees(Math.atan2(cxy[1] - y, cxy[0] - x));
                double err = Field.wrapDeg(bearing - (hDeg + 180));
                double d = Math.hypot(cxy[0] - x, cxy[1] - y);
                double score = Math.hypot(cxy[0] - lx, cxy[1] - ly);
                if (score < best) {
                    best = score;
                    current.aimedAt = which == 0 ? "near" : "far";
                    current.aimErrDeg = err;
                    current.dist = d;
                    current.targetWasUp = cellUp(cell);
                }
            }
            current.number = volleys.size() + 1;
            current.noTip = inList(s.noTip, current.number);
            volleys.add(current);
            ctx.event(String.format(Locale.US, "VOLLEY starts at our %s CELL (%s), aim off %+.1f deg, %.0f in. away, flywheel %.0f RPM",
                    current.aimedAt, current.targetWasUp ? "up" : "DOWN", current.aimErrDeg, current.dist, rpm));
        }
        Shot sh = new Shot();
        sh.number = ++ballsShot;
        sh.launchT = t;
        sh.landT = t + s.shotFlight;
        sh.rpm = rpm;
        double dir = Math.toRadians(hDeg + 180 + s.launcherYawErr);
        double range = Math.abs(rpm) < 800 ? 6 : s.shotRangeRef * sq(rpm / s.shotRpmRef);
        range += rng.nextGaussian() * s.shotSigmaRange;
        double lat = rng.nextGaussian() * s.shotSigmaLat;
        sh.x0 = x + s.halfLen * Math.cos(dir);  // leaves from the back of the robot
        sh.y0 = y + s.halfLen * Math.sin(dir);
        sh.lx = x + range * Math.cos(dir) - lat * Math.sin(dir);
        sh.ly = y + range * Math.sin(dir) + lat * Math.cos(dir);
        int heading = Field.nearCell(blue);  // the CELL of ours nearest where this ball comes down
        if (Math.hypot(Field.CELL_XY[Field.farCell(blue)][0] - sh.lx, Field.CELL_XY[Field.farCell(blue)][1] - sh.ly)
                < Math.hypot(Field.CELL_XY[heading][0] - sh.lx, Field.CELL_XY[heading][1] - sh.ly)) {
            heading = Field.farCell(blue);
        }
        sh.targetUpAtLaunch = cellUp(heading);
        if (!sh.targetUpAtLaunch) current.launchedAtDown++;
        sh.volley = current;
        current.balls++;
        current.rpmSum += rpm;
        current.lastLaunch = mt;
        flying.add(sh);
        rpm *= 1 - s.shotDip;
    }

    boolean cellUp(int cell) {
        return audienceUp[Field.hiveOf(cell)] == Field.isAudienceCell(cell);
    }

    int ballsShot = 0;

    /** "1,3" contains 3. */
    static boolean inList(String csv, int n) {
        for (String p : csv.split(",")) {
            if (p.trim().equals(String.valueOf(n))) return true;
        }
        return false;
    }

    void land(double t) {
        for (int i = flying.size() - 1; i >= 0; i--) {
            Shot sh = flying.get(i);
            if (sh.landT > t) continue;
            flying.remove(i);
            int cell = -1;
            for (int c = 0; c < 4; c++) {
                double[] cxy = Field.CELL_XY[c];
                if (Math.abs(sh.lx - cxy[0]) <= s.cellOpenHalfX && Math.abs(sh.ly - cxy[1]) <= s.cellOpenHalfY) cell = c;
            }
            Volley v = sh.volley;
            int outcome;
            if (cell < 0) {
                v.missed++;
                outcome = 0;
            } else if (!cellUp(cell)) {
                if (sh.targetUpAtLaunch) v.late++;
                else v.intoDown++;
                outcome = sh.targetUpAtLaunch ? 3 : 2;
            } else if (inList(s.missBalls, sh.number) || rng.nextDouble() < s.shotMiss) {
                v.bounced++;
                outcome = 4;
            } else {
                outcome = 1;
                v.intoUp++;
                if (v.noTip) {
                    if (v.intoUp == 1) ctx.event("balls go in, but the HIVE does NOT tip (scenario: noTip=" + s.noTip + ")");
                } else {
                    cellCount[cell]++;
                    int hive = Field.hiveOf(cell);
                    if (cellCount[cell] >= s.tipBalls && tipAt[hive] < 0) {
                        tipAt[hive] = t + s.tipDelay;
                        v.tipped = true;
                    }
                }
            }
            balls.add(new double[] {sh.launchT - s.initTime, sh.landT - s.initTime, sh.x0, sh.y0, sh.lx, sh.ly, outcome});
        }
    }

    void tips(double t, double mt) {
        for (int hive = 0; hive < 2; hive++) {
            if (tipAt[hive] >= 0 && t >= tipAt[hive]) {
                tipAt[hive] = -1;
                audienceUp[hive] = !audienceUp[hive];
                cellCount[hive * 2] = cellCount[hive * 2 + 1] = 0;
                boolean ours = hive == Field.hiveOf(Field.nearCell(blue));
                if (ours) {
                    ourTips++;
                    tipTimes.add(String.format(Locale.US, "%.1f", mt));
                } else {
                    otherTips++;
                }
                ctx.event((ours ? "OUR" : "THEIR") + " HIVE TIPPED: now our " + (cellUp(Field.nearCell(blue)) ? "near" : "far") + " CELL is up");
            }
        }
    }

    void events(double mt) {
        if (!partnerDone && s.partnerTip >= 0 && mt >= s.partnerTip) {
            partnerDone = true;
            int hive = Field.hiveOf(Field.nearCell(blue));
            if (tipAt[hive] < 0) {
                audienceUp[hive] = !audienceUp[hive];
                cellCount[hive * 2] = cellCount[hive * 2 + 1] = 0;
                partnerTips++;
                ctx.event("PARTNER tips our HIVE: now our " + (cellUp(Field.nearCell(blue)) ? "near" : "far") + " CELL is up");
            }
        }
        if (!bumpDone && s.bumpAt >= 0 && mt >= s.bumpAt) {
            bumpDone = true;
            x += blue ? -s.bumpX : s.bumpX;
            y += blue ? -s.bumpY : s.bumpY;
            hDeg += s.bumpH;
            ctx.event(String.format(Locale.US, "KNOCK: robot shoved %+.0f in. X, %+.0f in. Y, %+.0f deg", s.bumpX, s.bumpY, s.bumpH));
        }
    }

    void intake(double dt) {
        SimMotor im = ctx.intake;
        double p = im == null ? 0 : im.usefulPower();
        intakeTicks += p * 312 / 60.0 * 537.7 * dt;
        if (p < 0.3 || held >= 4) return;
        double h = Math.toRadians(hDeg), c = Math.cos(h), sn = Math.sin(h);
        for (double[] b : garden) {
            if (b[2] > 0 || held >= 4) continue;
            double dx = b[0] - x, dy = b[1] - y;
            double f = dx * c + dy * sn, r = dx * sn - dy * c;
            if (f >= s.halfLen - 3.6 && f <= s.halfLen + 1.5 && Math.abs(r) <= 6.0) {
                int n = garden.indexOf(b);
                if (inList(s.missPickups, n + 1) || rng.nextDouble() < s.intakeMiss) {
                    b[2] = 2;  // decided once: this one rolls away from the intake
                    gardenMissed[n] = ctx.matchTime();
                    ctx.event("intake MISSES a GARDEN POLLEN (" + held + " held)");
                    continue;
                }
                b[2] = 1;
                gardenTaken[n] = ctx.matchTime();
                if (held == 0) nextLaunchAt = siphonPos + 0.2;
                held++;
                ctx.event("intake picks up a GARDEN POLLEN (" + held + " held)");
            }
        }
    }

    void encoders() {
        for (int i = 0; i < 4; i++) {
            SimMotor m = ctx.wheels[i];
            if (m == null) continue;
            double inchPerTick = Math.PI * 96 * (1 + s.diamErr[i]) / 25.4 / 537.7;
            m.setTruth(travel[i] / inchPerTick, wr[i] / inchPerTick);
            // DC motor current: (applied voltage - back EMF) / resistance. goBILDA 5203 312 RPM: 9.2 A stall,
            // 0.25 A free, 61.7 in/s rim speed at 12 V with no load. Pushing something with the wheels
            // slipping slowly = high current; cruising = low.
            double drive = m.usefulPower() * volts / 12 - wr[i] / 61.7;
            m.setAmps(0.25 + Math.min(9.2, 9.2 * Math.abs(drive)));
        }
        if (ctx.flywheel != null) ctx.flywheel.setTruth(flyTicks, rpm * 28 / 60);
        if (ctx.intake != null) ctx.intake.setTruth(intakeTicks, 0);
    }

    // ================================================================ record

    public static final String TRACE_COLUMNS = "t,true_x,true_y,true_heading,odo_x,odo_y,odo_heading,slipping,held,flywheel_rpm,volts,"
            + "red_hive_audience_up,blue_hive_audience_up,tags_in_view,over_midline,touching_leg";

    void record(double mt) {
        double[] o = ours();
        double[] odo = ctx.odometry.get();
        // columns: see TRACE_COLUMNS
        double[] row = new double[] {mt, o[0], o[1], o[2], Double.NaN, Double.NaN, Double.NaN, slipping ? 1 : 0, held, rpm, volts,
                audienceUp[0] ? 1 : 0, audienceUp[1] ? 1 : 0,
                ctx.portal != null ? ctx.portal.latestTagCount() : ctx.camera.latestTagCount(), overMid ? 1 : 0,
                ctx.now() - lastLegContact < 0.06 ? 1 : 0};
        if (odo != null) {
            row[4] = odo[0];
            row[5] = odo[1];
            row[6] = ((odo[2] % 360) + 360) % 360;
            // While pressed on a wall the wheels slip and the code knows it (it resets on the wall), so only count driving.
            if (mt <= s.matchTime && ctx.now() - lastWallContact > 0.4) {
                double e = Math.hypot(odo[0] - o[0], odo[1] - o[1]);
                odoErrMax = Math.max(odoErrMax, e);
                odoErrSum += e;
                odoErrN++;
                odoHeadErrMax = Math.max(odoHeadErrMax, Math.abs(Field.wrapDeg(odo[2] - o[2])));
            }
        }
        trace.add(row);
        traceStep.add(ctx.currentStep());
    }

    void endOfAuto() {
        endPose = ours();
        endOdo = ctx.odometry.get();
        double gap = 1e9;
        for (double[] p : corners()) {
            gap = Math.min(gap, Math.min(Math.min(p[0], Field.SIZE - p[0]), Math.min(p[1], Field.SIZE - p[1])));
        }
        endWallGap = gap;
        endTouchingWall = gap < 0.3;
        // PARK: any part of the robot over the LOADING ZONE (GUESS rectangle, "our side")
        endInLoadingZone = false;
        double[][] cs = corners();
        for (int i = 0; i < 4 && !endInLoadingZone; i++) {
            for (int j = 0; j <= 10; j++) {
                double[] a = cs[i], b = cs[(i == 0 ? 1 : i == 1 ? 3 : i == 3 ? 2 : 0)];
                double px = a[0] + (b[0] - a[0]) * j / 10.0, py = a[1] + (b[1] - a[1]) * j / 10.0;
                double[] ou = Field.toOurs(blue, px, py, 0);
                if (ou[0] >= s.lzX0 && ou[0] <= s.lzX1 && ou[1] >= s.lzY0 && ou[1] <= s.lzY1) {
                    endInLoadingZone = true;
                    break;
                }
            }
        }
        double[] ou = Field.toOurs(blue, x, y, 0);
        if (ou[0] >= s.lzX0 && ou[0] <= s.lzX1 && ou[1] >= s.lzY0 && ou[1] <= s.lzY1) endInLoadingZone = true;
    }
}
