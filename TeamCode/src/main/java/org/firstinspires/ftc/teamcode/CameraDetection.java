package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.json.JSONObject;

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.net.HttpURLConnection;
import java.net.InetAddress;
import java.net.URL;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.TreeSet;

/*
 * CAMERA DETECTION: a test that only LOOKS. It moves nothing (no motors, no servos).
 * It shows on the Driver Hub what the Limelight recognises:
 *   - the AprilTags under the HIVE CELLS (which CELL, how far behind the robot, how far to the side)
 *   - YELLOW POLLEN, RED NECTAR and BLUE NECTAR (how many, where the biggest one is, roughly how far)
 *
 * HOW TO USE: pick "Camera Detection" (TeleOp list), press INIT, read the screen. START is not
 * needed: the screen is the same before and after START.
 *   dpad left / right   change the LOOK: ALL -> TAGS -> YELLOW -> RED -> BLUE -> ALL ...
 *   Y                   show the settings the camera has stored for the look it is on (Y again = back)
 * The Limelight can only look for ONE thing at a time (it has numbered "slots", one per thing).
 * ALL takes turns: about half a second for each, so every line is a few seconds old at most and
 * says how old. Holding on one look keeps the camera on that slot, so the picture is steady:
 * use that with the Camera Stream (Driver Station menu, INIT only) or the Limelight web page.
 * The Camera Stream is ONE picture per tap, from the slot the camera is on at that moment. So to
 * see which tags get a green box, hold on TAGS first: on ALL, 3 pictures in 4 come from a color
 * slot and show no boxes at all. The Limelight writes two small numbers in the top left corner of
 * its picture: pictures per second, and under it (as far as seen on two photos) the slot: 1 = tags.
 *
 * THE CAMERA is on the BACK of the robot, next to the launcher, looking backward. So what is on
 * the RIGHT of its picture is on the robot's LEFT. Everything on the screen is already turned into
 * ROBOT terms: "left" and "right" mean the robot's own left and right (as if you sat in it facing
 * its intake), and "behind" means behind the robot.
 *
 * WHAT THE SCREEN SHOWS
 *   Camera: ...          problems first (no "limelight" in the Robot Configuration, NO PICTURES,
 *                        a slot the camera won't switch to); else pictures per second and the look.
 *   HIVE TAGS: ...       one block per CELL seen: its tags, inches behind, degrees left / right, and
 *                        whether Auto3 would shoot at that CELL. The maths is Auto3's, so the numbers
 *                        are the ones Auto3 gets. Tags that are not HIVE tags: "other tag N".
 *                        "the camera finds no tag of: ..." names the CELLS it did not find. All 16
 *                        tags (30-45, red and blue) are looked for the same way, so a CELL in that
 *                        line is one whose tags the CAMERA did not find (no green box on them in the
 *                        Camera Stream): they are too small in the picture (far away), seen too much
 *                        from the side, or hidden. Go closer or face that CELL: from the red side the
 *                        red CELLS are the near ones. TAG_DOWNSCALE below is how small it still finds.
 *   YELLOW POLLEN: ...   how many blobs of that color, in how many of the pictures (20 of 20 = steady,
 *   RED NECTAR: ...      3 of 20 = flickering: tune the color), then for the BIGGEST blob: degrees
 *   BLUE NECTAR: ...     left / right, degrees up / down, how much of the picture it fills, and a ROUGH
 *                        distance worked out from its size (so half a ball hidden = reads too far).
 *   SETUP                per slot: whether the numbers in this file are on the camera.
 *
 * HOW TO TUNE A COLOR
 *   1. Plug the Limelight into a laptop with its USB cable and open http://limelight.local:5801
 *      (or http://172.29.0.1:5801) in a browser. That is the Limelight's own web page.
 *   2. Pick the slot of the color (YELLOW 2, RED 3, BLUE 4: see the color table below).
 *   3. Put a ball in front of the camera, in the light of the room you will play in. If the ball
 *      looks white, the picture is too bright: lower the exposure. If it is nearly black: raise it.
 *   4. Use the eyedropper on the ball, then move the hue / saturation / value sliders until the ball
 *      is found and the floor, walls and people are not.
 *   5. Type the hue / sat / val numbers into the color table below, and the exposure into
 *      COLOR_EXPOSURE. They are then sent to the camera at every INIT, on any Limelight.
 *      (Or set WRITE_SETTINGS = false: then this file never changes the camera and the tuning you
 *      did on the web page stays. With WRITE_SETTINGS = true the web page tuning is OVERWRITTEN by
 *      the table at the next INIT.)
 *   No laptop: hold on the color with the dpad, open the Camera Stream, change the table, try again.
 *
 * FOR AUTO3 / AUTO4 LATER: the recognising itself is in small pieces at the bottom of this file
 * that take one Limelight picture (an LLResult) and give back plain numbers. They keep nothing of
 * this OpMode, so any OpMode can call them:
 *   CameraDetection.findBlobs(result, smallestPercent, ballInches)   the color blobs in a picture
 *   new CameraDetection.TagLook(), add(result) per picture, cells()  the HIVE CELLS in view
 * NOTE: switching the Limelight to another slot (pipelineSwitch) makes the program WAIT for the
 * camera to answer. Here the robot stands still, so that is fine. An auto must NOT do it while
 * the robot is driving: it would drive blind for that time.
 */
@TeleOp(name = "Camera Detection", group = "Test")
public class CameraDetection extends LinearOpMode {

    // ================================================================ THE COLOR TABLE
    // One row per color the camera looks for. Change the numbers, build, press INIT.
    //   name, what it is   words for the screen
    //   slot               the Limelight slot (2 to 9) that looks for this color. Slot 0 is Auto4's
    //                      color picture and slot 1 is the AprilTags: never use those two here.
    //   hue from, to       WHICH color, 0-179: red 0, yellow 30, green 60, blue 120, and red again at 179
    //   outside            true = take the hues OUTSIDE from..to. Red needs that: red is at BOTH ends
    //                      of the scale (below 10 and above 170).
    //   sat from, to       how STRONG the color is, 0-255 (0 = white or grey, 255 = pure color)
    //   val from, to       how BRIGHT it is, 0-255 (0 = black)
    //   ball               inches across (POLLEN 2.8, NECTAR 3.6, as in Auto4), for the rough distance
    static final ColorRow[] COLORS = {
            //           name      what it is  slot  hue from, to  outside  sat from, to  val from, to  ball
            new ColorRow("YELLOW", "POLLEN",   2,     20,  35,     false,   100, 255,     100, 255,    2.8),
            new ColorRow("RED",    "NECTAR",   3,     10, 170,     true,    120, 255,      80, 255,    3.6),
            new ColorRow("BLUE",   "NECTAR",   4,    100, 125,     false,   100, 255,      60, 255,    3.6),
    };

    // true  = the numbers in this file are sent to the camera at INIT (only when they differ from
    //         what the camera has stored, so a normal INIT changes nothing on the camera).
    // false = the camera is left exactly as it is; SETUP only says where it differs from this file.
    static final boolean WRITE_SETTINGS = true;

    // Brightness of the picture for the color slots, as on the Limelight web page. They start the
    // same as Auto3's AprilTag numbers; type your own numbers here to make them different.
    // Bigger exposure = brighter picture, but more blur when something moves.
    static final double COLOR_EXPOSURE = Auto3.TAG_EXPOSURE;
    static final double COLOR_GAIN = Auto3.TAG_GAIN;

    // Blobs filling less of the picture than this (in percent) are ignored: specks and reflections.
    // 0.05 is about a POLLEN 11 ft away.
    static final double MIN_BLOB_PERCENT = 0.05;

    static final double LOOK_TIME = 0.5;    // seconds the camera stays on one look when the look is ALL
    static final double SWITCH_WAIT = 1.5;  // seconds the camera gets to switch slot before it counts as a problem
    static final boolean SHOW_PICTURE = true;  // the Limelight's picture on the Driver Station Camera Stream (INIT only)

    // TWO THINGS THAT ARE BELIEVED, NOT YET CHECKED ON THE ROBOT (the test below takes a minute):
    // AREA_FULL: the number the Limelight gives for a blob filling the WHOLE picture: 1 or 100.
    //            Hold a ball 24 in. from the lens: if "roughly ... in. away" is 10 times too far
    //            or too close, change this to 100.
    // BLOB_UP:   hold a ball HIGH in the picture: if the screen says "down", change this to -1.
    static final double AREA_FULL = 1.0;
    static final double BLOB_UP = 1;

    // The camera's angle of view in degrees. Same as Auto4 (CAMERA_HFOV, CAMERA_VFOV), keep equal.
    static final double CAMERA_HFOV = 54.5, CAMERA_VFOV = 42.0;
    // The HIVE tags. Same as Auto3 (in its setUp, readTags and findCell), keep equal.
    static final double TAG_SIZE_MM = 82.55;        // the 3.25 in. HIVE tags
    static final double TAG_TOO_FAR = 90;           // inches: a tag further behind than this is not trusted
    static final int FIRST_TAG = 30, LAST_TAG = 45; // 4 CELLS x 4 tags
    // HOW SMALL A TAG THE CAMERA STILL FINDS. A far CELL, or one seen from the side, is only a few
    // pixels in the picture. To be fast the camera shrinks the picture before it searches it:
    // 2 = to half size, 1 = not shrunk. With 1 it finds tags about half as big (about twice as far)
    // and gives fewer pictures a second. Choices: 1, 1.5, 2, 3, 4. It is the "Detector Downscale"
    // on the Limelight web page. It is stored in the tag slot, the one the autos use, so they get
    // it too. BELIEVED, NOT YET CHECKED: its name on the camera (fiducial_resdiv, in setUp). The
    // SETUP line of slot 1 says whether the camera took it.
    static final double TAG_DOWNSCALE = 1;
    // From Auto3 itself, so they can never differ: CAMERA_NAME, PIPE_TAGS, TAG_EXPOSURE, TAG_GAIN,
    // CAMERA_BACK, CAMERA_PITCH, AIM_TRIM, AIM_MAX, FAR_CELL, CELLS, TAG_PLACE.

    // Which stored settings Y shows: every setting whose name has one of these in it.
    static final String[] SHOWN_SETTINGS = {"type", "desc", "hue", "sat_", "val_", "area", "exposure", "gain",
            "black", "size", "3d", "fiducial", "_res"};

    /** One row of the color table. */
    public static final class ColorRow {
        public final String name, what;
        public final int slot;
        public final double hueFrom, hueTo, satFrom, satTo, valFrom, valTo, ballInches;
        public final boolean outside;

        ColorRow(String name, String what, int slot, double hueFrom, double hueTo, boolean outside,
                 double satFrom, double satTo, double valFrom, double valTo, double ballInches) {
            this.name = name;
            this.what = what;
            this.slot = slot;
            this.hueFrom = hueFrom;
            this.hueTo = hueTo;
            this.outside = outside;
            this.satFrom = satFrom;
            this.satTo = satTo;
            this.valFrom = valFrom;
            this.valTo = valTo;
            this.ballInches = ballInches;
        }
    }

    /** One thing the camera can look for: the HIVE tags (row == null) or one color. */
    static class Look {
        final String name, title;
        final int slot;
        final ColorRow row;
        String setup = "not set up";  // what INIT did with this slot's settings
        // The last finished look, for the screen.
        final List<String> lines = new ArrayList<>();
        final ElapsedTime age = new ElapsedTime();
        // The look going on now.
        int pictures, withBlobs;
        boolean overFull;
        List<Blob> blobs = new ArrayList<>();
        final TagLook tags = new TagLook();

        Look(String name, String title, int slot, ColorRow row) {
            this.name = name;
            this.title = title;
            this.slot = slot;
            this.row = row;
            lines.add("not looked at yet");
        }
    }

    Limelight3A limelight;  // null: not in the Robot Configuration
    Picture picture;        // the Camera Stream, or null
    final List<Look> looks = new ArrayList<>();  // the TAGS first, then one per color row
    final List<String> tableProblems = new ArrayList<>();
    int hold = -1;          // -1 = ALL (take turns), else the place in looks it stays on
    int now = 0;            // the place in looks of the look going on now
    boolean asked = false;  // the camera has been asked for this look's slot
    boolean waiting = false;  // ... and its first picture from that slot hasn't come yet
    final ElapsedTime timer = new ElapsedTime();         // time in this step of the look
    final ElapsedTime sincePicture = new ElapsedTime();  // time since the last new picture
    final ElapsedTime rateTimer = new ElapsedTime();
    double lastStamp = -1;  // the Limelight's time stamp of the last picture counted, so each counts once
    int slotNow = -1;       // the slot the last picture came from
    int counted, rate;      // pictures this second, pictures in the last whole second
    String slotProblem = "";
    List<String> stored;    // Y: the settings stored on the camera, or null = the normal screen

    @Override
    public void runOpMode() {
        setUp();
        while (opModeInInit() || opModeIsActive()) {  // the same before and after START
            if (limelight != null) {
                buttons();
                look();
            }
            if (isStarted()) {
                stopPicture();  // the Driver Station only shows the Camera Stream during INIT
            }
            show();
            sleep(5);
        }
        stopPicture();
        if (limelight != null) {
            if (sincePicture.seconds() < 1) {
                limelight.pipelineSwitch(Auto3.PIPE_TAGS);  // leave the camera on the tags, where the autos expect it
            }
            limelight.stop();
        }
    }

    void stopPicture() {
        if (picture != null) {
            picture.stop();
            picture = null;
            CameraStreamServer.getInstance().setSource(null);
        }
    }

    // ================================================================ LOOKING

    void buttons() {
        boolean left = gamepad1.dpadLeftWasPressed() | gamepad2.dpadLeftWasPressed();  // single | so both are read
        boolean right = gamepad1.dpadRightWasPressed() | gamepad2.dpadRightWasPressed();
        if (left != right) {
            int choices = looks.size() + 1;  // ALL, then each look
            hold = (hold + 1 + (right ? 1 : choices - 1)) % choices - 1;
            if (hold >= 0) {
                now = hold;
            }
            asked = false;  // ask the camera for the slot again
            stored = null;
        }
        if (gamepad1.yWasPressed() | gamepad2.yWasPressed()) {
            stored = stored != null ? null : storedSettings(looks.get(now));
        }
    }

    /**
     * One step of looking; never waits, except for the one call that switches the slot.
     * A look goes: ask the camera for the look's slot -> wait for the first picture that really
     * comes from that slot -> count every NEW picture from that slot for LOOK_TIME -> put the
     * result on the screen -> (ALL) on to the next look, or (holding) the same look again.
     */
    void look() {
        LLResult result = limelight.getLatestResult();
        boolean fresh = result != null && result.getTimestamp() != lastStamp;
        if (fresh) {
            lastStamp = result.getTimestamp();
            slotNow = result.getPipelineIndex();
            sincePicture.reset();
            counted++;
        }
        if (rateTimer.seconds() >= 1) {
            rate = counted;
            counted = 0;
            rateTimer.reset();
        }
        Look look = looks.get(now);
        if (!asked) {
            if (sincePicture.seconds() > 1) {
                return;  // no pictures at all: nothing to ask (the Camera line says so)
            }
            if (slotNow != look.slot) {
                // The one call here that WAITS for the camera to answer. Fine in this test: the robot
                // stands still. An auto must not switch slots while it is driving.
                limelight.pipelineSwitch(look.slot);
            }
            asked = true;
            waiting = true;
            timer.reset();
            return;
        }
        if (waiting) {
            if (fresh && slotNow == look.slot) {
                // The first picture from the new slot. The look starts with the NEXT one, so nothing
                // from the moment of switching is counted.
                waiting = false;
                slotProblem = "";
                look.pictures = look.withBlobs = 0;
                look.overFull = false;
                look.tags.clear();
                timer.reset();
            } else if (timer.seconds() > SWITCH_WAIT) {
                if (sincePicture.seconds() < 1) {  // pictures do come, but from another slot
                    slotProblem = "asked for slot " + look.slot + " (" + look.name + "), the Limelight stays on slot " + slotNow;
                    look.lines.clear();
                    look.lines.add("NOT LOOKED AT: the Limelight won't switch to slot " + look.slot);
                    look.age.reset();
                }
                asked = false;  // ask again
                if (hold < 0) {
                    now = (now + 1) % looks.size();  // ALL: one bad slot must not stop the other looks
                }
            }
            return;
        }
        if (fresh) {
            if (slotNow != look.slot) {
                asked = false;  // something else switched the camera (the web page?): ask again
                return;
            }
            look.pictures++;
            if (look.row == null) {
                look.tags.add(result);
            } else {
                List<Blob> found = findBlobs(result, MIN_BLOB_PERCENT, look.row.ballInches);
                if (!found.isEmpty()) {
                    look.withBlobs++;
                    look.blobs = found;
                    look.overFull |= found.get(0).percent > 100;
                }
            }
        }
        if (timer.seconds() >= LOOK_TIME) {
            finish(look);
            if (hold < 0 && looks.size() > 1) {
                now = (now + 1) % looks.size();
                asked = false;
            } else {  // holding: the same look again, the camera stays on its slot
                look.pictures = look.withBlobs = 0;
                look.overFull = false;
                look.tags.clear();
                timer.reset();
            }
        }
    }

    /** Turns what a look counted into its lines for the screen. */
    void finish(Look look) {
        look.lines.clear();
        look.age.reset();
        if (look.pictures == 0) {
            look.lines.add("no pictures during this look");
        } else if (look.row == null) {
            List<Cell> cells = look.tags.cells();
            look.lines.add(cells.isEmpty() ? "no HIVE tags in view"
                    : cells.size() + (cells.size() == 1 ? " CELL" : " CELLS") + ", " + look.pictures + " pictures");
            for (Cell cell : cells) {
                StringBuilder ids = new StringBuilder();
                for (int t = 0; t < 4; t++) {
                    if (cell.used[t]) {
                        ids.append(' ').append(cell.firstId + t);
                    }
                }
                look.lines.add(cell.name + " CELL: tags" + (cell.tags == 0 ? " none usable" : ids) + " (" + cell.tags + " of 4)");
                if (cell.tags > 0) {
                    look.lines.add(String.format("  %.1f in. behind, %s", cell.behind, side(cell.degreesRight)));
                    look.lines.add(cell.auto3Target
                            ? String.format("  Auto3 shoots at this one: aim %.1f deg off%s", Math.abs(cell.aimOff),
                                    cell.tags < 2 ? " (1 tag: too rough to turn on)" : "")
                            : "  not Auto3's CELL: " + cell.auto3Why);
                }
                if (!cell.notUsed.isEmpty()) {
                    look.lines.add("  seen but not used:" + cell.notUsed);
                }
            }
            for (int id : look.tags.otherTags) {
                look.lines.add("other tag " + id);
            }
            // The CELLS with no tag at all in these pictures: the camera did not find them.
            StringBuilder notFound = new StringBuilder();
            for (String name : Auto3.CELLS) {
                boolean found = false;
                for (Cell cell : cells) {
                    found |= cell.name.equals(name);
                }
                if (!found) {
                    notFound.append(notFound.length() == 0 ? "" : ", ").append(name);
                }
            }
            if (!cells.isEmpty() && notFound.length() > 0) {
                look.lines.add("the camera finds no tag of: " + notFound);
                look.lines.add("  (too small in the picture, too sideways, or hidden)");
            }
        } else if (look.withBlobs == 0) {
            look.lines.add("none (" + look.pictures + " pictures)");
        } else {
            Blob biggest = look.blobs.get(0);
            look.lines.add(String.format("%d blob%s, in %d of %d pictures", look.blobs.size(),
                    look.blobs.size() == 1 ? "" : "s", look.withBlobs, look.pictures));
            look.lines.add(String.format("biggest: %s, %.1f deg %s", side(biggest.degreesRight),
                    Math.abs(biggest.degreesUp), biggest.degreesUp >= 0 ? "up" : "down"));
            look.lines.add(String.format("%.2f%% of the picture, roughly %.0f in. away", biggest.percent, biggest.inches));
            if (look.overFull) {
                look.lines.add("A blob bigger than the picture: set AREA_FULL to 100");
            }
        }
    }

    /** "3.2 deg left": which side of straight behind, in the ROBOT's left and right. */
    static String side(double degreesRight) {
        return String.format("%.1f deg %s", Math.abs(degreesRight), degreesRight >= 0 ? "right" : "left");
    }

    // ================================================================ THE SCREEN

    void show() {
        if (limelight == null) {
            telemetry.addLine("Camera: NO device named \"" + Auto3.CAMERA_NAME + "\" in the Robot Configuration");
            telemetry.addLine("(Configure Robot: it is an Ethernet Device, shown after Scan.)");
            telemetry.update();
            return;
        }
        Look current = looks.get(now);
        telemetry.addLine(sincePicture.seconds() > 1 ? "Camera: NO PICTURES from the Limelight (cable? power?)"
                : !slotProblem.isEmpty() ? "Camera: WRONG SLOT: " + slotProblem
                : String.format("Camera: %d pictures a second. Look: %s", rate,
                        hold < 0 ? "ALL (now " + current.name + ")" : current.name + " only"));
        telemetry.addLine("dpad left/right: next look.  Y: settings");
        if (stored != null) {
            for (String line : stored) {
                telemetry.addLine(line);
            }
            telemetry.update();
            return;
        }
        telemetry.addLine("left/right = the ROBOT's (camera looks back)");
        for (Look look : looks) {
            telemetry.addLine(look.title + ": " + look.lines.get(0)
                    + (look.age.seconds() >= 1 && !look.lines.get(0).startsWith("not looked")
                            ? String.format("  (%.0f s ago)", look.age.seconds()) : ""));
            for (int i = 1; i < look.lines.size(); i++) {
                telemetry.addLine("   " + look.lines.get(i));
            }
        }
        telemetry.addLine("SETUP (WRITE_SETTINGS is " + (WRITE_SETTINGS ? "on" : "off") + ")");
        for (Look look : looks) {
            telemetry.addLine("  slot " + look.slot + " " + look.name + ": " + look.setup);
        }
        for (String problem : tableProblems) {
            telemetry.addLine("  " + problem);
        }
        if (picture != null && opModeInInit()) {
            // On ALL the camera is on the tag slot a quarter of the time only, and the Camera Stream
            // is ONE picture per tap: mostly a picture from a color slot, with no boxes on the tags.
            telemetry.addLine(hold < 0 ? "Picture: FIRST dpad to ONE look (on ALL it is from any look), then menu > Camera Stream"
                    : "Picture: menu > Camera Stream (INIT only), tap it for a new one");
        }
        telemetry.update();
    }

    /** Y: the settings the camera has stored in this look's slot, read from the camera itself. */
    List<String> storedSettings(Look look) {
        List<String> lines = new ArrayList<>();
        lines.add("ON THE CAMERA, slot " + look.slot + " (" + look.name + ").  Y = back");
        JSONObject whole = readSlot(look.slot);
        if (whole == null) {
            lines.add("could not read this slot from the camera");
            return lines;
        }
        TreeSet<String> names = new TreeSet<>();
        for (Iterator<String> keys = whole.keys(); keys.hasNext(); ) {
            String key = keys.next();
            for (String word : SHOWN_SETTINGS) {
                if (key.contains(word)) {
                    names.add(key);
                    break;
                }
            }
        }
        for (String key : names) {
            String value = String.valueOf(whole.opt(key));
            if (lines.size() < 45) {
                lines.add("  " + key + " = " + (value.length() > 30 ? value.substring(0, 30) + "..." : value));
            }
        }
        lines.add("(" + names.size() + " shown of " + whole.length() + " settings)");
        return lines;
    }

    // ================================================================ SETUP

    void setUp() {
        looks.add(new Look("TAGS", "HIVE TAGS", Auto3.PIPE_TAGS, null));
        for (ColorRow row : COLORS) {
            boolean taken = false;
            for (Look look : looks) {
                taken |= look.slot == row.slot;
            }
            if (row.slot < 2 || row.slot > 9 || taken) {
                tableProblems.add(row.name + " is NOT looked for: slot " + row.slot + " is "
                        + (taken ? "used twice" : "not allowed") + " (use 2 to 9, each once; 0 and 1 are Auto4's and the tags)");
            } else {
                looks.add(new Look(row.name, row.name + " " + row.what, row.slot, row));
            }
        }
        limelight = hardwareMap.tryGet(Limelight3A.class, Auto3.CAMERA_NAME);
        if (limelight == null) {
            return;
        }
        telemetry.addLine("Setting up the camera...");
        telemetry.update();
        limelight.setPollRateHz(100);
        limelight.start();
        // Pictures first: with no pictures the camera is off or unplugged, and setting it up would
        // only be a long wait.
        double first = stamp();
        ElapsedTime wait = new ElapsedTime();
        while (!isStopRequested() && wait.seconds() < 2 && stamp() == first) {
            sleep(20);
        }
        boolean pictures = stamp() != first;
        for (Look look : looks) {
            if (isStopRequested()) {
                return;
            }
            if (!pictures) {
                look.setup = "not checked: no pictures. Press INIT again once the camera works";
                continue;
            }
            telemetry.addLine("Setting up the camera: slot " + look.slot + " " + look.name + " ...");
            telemetry.update();
            ColorRow row = look.row;
            // The tag slot gets what Auto3 gives it (same as Auto3.setUp, keep equal), and TAG_DOWNSCALE.
            look.setup = row == null
                    ? setUpSlot(look.slot, "pipe_fiducial", json("desc", "tags", "fiducial_size", TAG_SIZE_MM,
                            "exposure", Auto3.TAG_EXPOSURE, "sensor_gain", Auto3.TAG_GAIN,
                            "fiducial_resdiv", TAG_DOWNSCALE))
                    : setUpSlot(look.slot, "pipe_color", json("desc", row.name,
                            "hue_min", row.hueFrom, "hue_max", row.hueTo, "invert_hue", row.outside ? 1 : 0,
                            "sat_min", row.satFrom, "sat_max", row.satTo, "val_min", row.valFrom, "val_max", row.valTo,
                            "exposure", COLOR_EXPOSURE, "sensor_gain", COLOR_GAIN));
        }
        if (SHOW_PICTURE) {
            picture = new Picture(limelightAddress());
            CameraStreamServer.getInstance().setSource(picture);
        }
        lastStamp = stamp();
        sincePicture.reset();
        rateTimer.reset();
    }

    double stamp() {
        LLResult result = limelight.getLatestResult();
        return result == null ? -1 : result.getTimestamp();
    }

    /**
     * Makes one slot of the camera what this file says: the type (pipe_fiducial = AprilTags,
     * pipe_color = a color) and the settings in wanted. Returns what happened, for the SETUP lines.
     * It first READS the slot back from the camera and only sends something when the camera has
     * other numbers, so a normal INIT changes nothing on the camera. After sending it reads the
     * slot again, to see that the camera kept the numbers.
     * A setting the camera's slot does not have at all is never sent; SETUP names it (and settings
     * with a similar name the camera does have), because then the name in this file is wrong.
     */
    String setUpSlot(int slot, String type, JSONObject wanted) {
        JSONObject stored = readSlot(slot);
        if (stored == null) {
            // Could not read the slot: do it the way Auto3 does, without checking.
            if (!WRITE_SETTINGS) {
                return "could not read it from the camera; left as it is";
            }
            if (!goToSlot(slot)) {
                return "NOT SET: the camera did not switch to this slot";
            }
            String got = makeType(slot, type);
            boolean sent = setPipeline(wanted, true);
            return !type.equals(got) ? "is \"" + got + "\", NOT " + type + ": nothing will be found"
                    : sent ? "sent, but could not read it back to check" : "NOT SET: the camera refused the settings";
        }
        List<String> unknown = new ArrayList<>();
        List<String> differ = differences(wanted, stored, unknown);
        boolean typeOk = type.equals(stored.optString("pipeline_type"));
        StringBuilder note = new StringBuilder();
        for (String key : unknown) {
            note.append(". The camera has NO setting \"").append(key).append('"').append(similar(key, stored));
        }
        if (!WRITE_SETTINGS) {
            return (typeOk && differ.isEmpty() ? "left as it is (same as this file)"
                    : "left as it is. Differs from this file:" + (typeOk ? "" : " type") + join(differ)) + note;
        }
        if (typeOk && differ.isEmpty()) {
            return "ok, already on the camera" + note;
        }
        // Something to change. Settings always go to the slot the camera is running, so it must
        // really be on this slot first: otherwise they would land in another slot (the tags!).
        if (!goToSlot(slot)) {
            return "NOT SET: the camera did not switch to this slot" + note;
        }
        boolean typeChanged = !type.equals(pipelineType());
        String got = typeChanged ? makeType(slot, type) : type;
        if (!type.equals(got)) {
            return "is \"" + got + "\", NOT " + type + ": nothing will be found" + note;
        }
        if (differ.isEmpty() && !typeChanged) {
            return "ok" + note;  // the camera says the type is right, and so are the numbers
        }
        // Send every setting the camera knows (a slot whose type just changed may have lost some).
        JSONObject send = new JSONObject();
        for (Iterator<String> keys = wanted.keys(); keys.hasNext(); ) {
            String key = keys.next();
            if (!unknown.contains(key)) {
                try {
                    send.put(key, wanted.opt(key));
                } catch (Exception e) {
                    // can't happen: the value came out of a JSONObject
                }
            }
        }
        boolean sent = setPipeline(send, true);
        sleep(300);
        JSONObject after = readSlot(slot);
        if (!sent || after == null) {
            return (sent ? "sent, but could not read it back to check" : "NOT SET: the camera refused the settings") + note;
        }
        List<String> notKept = differences(wanted, after, new ArrayList<String>());
        return (notKept.isEmpty() ? "stored (changed:" + (typeChanged ? " type" : "") + join(differ) + ")"
                : "the camera did NOT keep:" + join(notKept)) + note;
    }

    /** The settings in wanted that the slot has with another value; into unknown: those it doesn't have at all. */
    static List<String> differences(JSONObject wanted, JSONObject stored, List<String> unknown) {
        List<String> differ = new ArrayList<>();
        for (Iterator<String> keys = wanted.keys(); keys.hasNext(); ) {
            String key = keys.next();
            if (!stored.has(key)) {
                unknown.add(key);
                continue;
            }
            Object want = wanted.opt(key);
            Object have = stored.opt(key);
            if (have instanceof Boolean) {
                have = (Boolean) have ? 1 : 0;  // a camera that says true / false for 1 / 0
            }
            boolean same;
            if (want instanceof Number) {
                try {
                    same = Math.abs(((Number) want).doubleValue() - Double.parseDouble(String.valueOf(have))) < 0.011;
                } catch (NumberFormatException e) {
                    same = false;
                }
            } else {
                same = String.valueOf(want).equals(String.valueOf(have));
            }
            if (!same) {
                differ.add(key);
            }
        }
        return differ;
    }

    /** For a setting the camera doesn't have: the settings it does have with a word of that name in them. */
    static String similar(String key, JSONObject stored) {
        StringBuilder found = new StringBuilder();
        for (String word : key.split("_")) {
            if (word.length() < 3 || word.equals("min") || word.equals("max")) {
                continue;
            }
            for (Iterator<String> keys = stored.keys(); keys.hasNext(); ) {
                String name = keys.next();
                if (name.contains(word) && found.indexOf(" " + name) < 0 && found.length() < 60) {
                    found.append(' ').append(name);
                }
            }
        }
        return found.length() == 0 ? "" : " (it has:" + found + ")";
    }

    static String join(List<String> words) {
        StringBuilder all = new StringBuilder();
        for (String word : words) {
            all.append(' ').append(word);
        }
        return all.toString();
    }

    /** A JSONObject from name, value, name, value ... */
    static JSONObject json(Object... namesAndValues) {
        JSONObject object = new JSONObject();
        try {
            for (int i = 0; i + 1 < namesAndValues.length; i += 2) {
                object.put((String) namesAndValues[i], namesAndValues[i + 1]);
            }
        } catch (Exception e) {
            // can't happen with the plain numbers and words used here
        }
        return object;
    }

    // ================================================================ LIMELIGHT
    // These WAIT for the camera to answer. They are only used at INIT and when a button is pressed.

    /** Switches the camera to a slot and waits (3 s at most) until a picture really comes from it. */
    boolean goToSlot(int slot) {
        double before = stamp();
        limelight.pipelineSwitch(slot);
        ElapsedTime wait = new ElapsedTime();
        while (!isStopRequested() && wait.seconds() < 3) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.getTimestamp() != before && result.getPipelineIndex() == slot) {
                sleep(300);  // let it settle on the new slot
                return true;
            }
            sleep(20);
        }
        return false;
    }

    /**
     * Makes the slot the camera is running the wanted type, if it isn't already, and returns the
     * type the Limelight itself then reports. Same as the middle of Auto3.makePipeline, keep equal.
     */
    String makeType(int slot, String type) {
        if (!type.equals(pipelineType())) {
            setPipeline(json("pipeline_type", type), true);
            limelight.reloadPipeline();  // a changed type only starts after a reload
            sleep(1000);
        }
        if (!type.equals(pipelineType())) {
            // Not taken: send the Limelight its own whole pipeline back with only the type changed.
            try {
                JSONObject whole = readSlot(slot);
                whole.put("pipeline_type", type);
                limelight.uploadPipeline(whole.toString(), slot);
                limelight.reloadPipeline();
                sleep(1000);
            } catch (Exception e) {
                // it keeps the type it had; SETUP shows which
            }
        }
        return pipelineType();
    }

    /** The type of the slot the Limelight is running now, as the Limelight reports it (3 tries). */
    String pipelineType() {
        String type = "unknown";
        for (int i = 0; i < 3 && !isStopRequested(); i++) {
            try {
                type = String.valueOf(limelight.getStatus().getPipelineType());
            } catch (Exception e) {
                type = "unknown";
            }
            if (type.startsWith("pipe_")) {
                break;
            }
            sleep(100);  // no answer that time
        }
        return type;
    }

    /**
     * Changes settings of the slot the Limelight is running now (save = keep after a restart).
     * The SDK has this but keeps it private, so it is called by name. Same as Auto3.setPipeline, keep equal.
     */
    boolean setPipeline(JSONObject settings, boolean save) {
        try {
            Method update = Limelight3A.class.getDeclaredMethod("updatePipeline", JSONObject.class, boolean.class);
            update.setAccessible(true);
            return (Boolean) update.invoke(limelight, settings, save);
        } catch (Exception e) {
            return false;
        }
    }

    /**
     * Reads everything the camera has stored in a slot (3 tries), or null if it can't.
     * Also private in the SDK and called by name (Auto3.makePipeline uses the same call).
     */
    JSONObject readSlot(int slot) {
        for (int i = 0; i < 3 && !isStopRequested(); i++) {
            try {
                Method get = Limelight3A.class.getDeclaredMethod("getPipelineAtIndex", int.class);
                get.setAccessible(true);
                JSONObject whole = (JSONObject) get.invoke(limelight, slot);
                if (whole != null && whole.length() > 0) {
                    return whole;
                }
            } catch (Exception e) {
                return null;  // this SDK doesn't have the call
            }
            sleep(100);
        }
        return null;
    }

    /** The Limelight's network address: the SDK keeps it inside, else the usual one. Same as Auto4, keep equal. */
    String limelightAddress() {
        try {
            Field field = Limelight3A.class.getDeclaredField("inetAddress");
            field.setAccessible(true);
            return ((InetAddress) field.get(limelight)).getHostAddress();
        } catch (Exception e) {
            return "172.29.0.1";
        }
    }

    /**
     * Reads the Limelight's video (port 5800, a stream of JPEG pictures) in the background and passes
     * the newest picture on to the Driver Station Camera Stream. It is the Limelight's own picture
     * of the slot it is on. Same as the reading part of Auto4's Camera, keep equal.
     */
    static class Picture implements CameraStreamSource {
        private final String url;
        private volatile boolean running = true;
        private volatile byte[] newest;

        Picture(String address) {
            url = "http://" + address + ":5800";
            Thread thread = new Thread(this::read);
            thread.setDaemon(true);
            thread.start();
        }

        void stop() {
            running = false;
        }

        private void read() {
            while (running) {
                try {
                    HttpURLConnection connection = (HttpURLConnection) new URL(url).openConnection();
                    connection.setConnectTimeout(2000);
                    connection.setReadTimeout(2000);
                    InputStream in = new BufferedInputStream(connection.getInputStream());
                    ByteArrayOutputStream jpeg = new ByteArrayOutputStream();
                    boolean inPicture = false;
                    int previous = -1;
                    int b;
                    while (running && (b = in.read()) != -1) {
                        if (!inPicture && previous == 0xFF && b == 0xD8) {  // a JPEG starts
                            inPicture = true;
                            jpeg.reset();
                            jpeg.write(0xFF);
                        }
                        if (inPicture) {
                            jpeg.write(b);
                            if (previous == 0xFF && b == 0xD9) {  // and ends
                                inPicture = false;
                                newest = jpeg.toByteArray();
                            }
                        }
                        previous = b;
                    }
                    connection.disconnect();
                } catch (Exception e) {
                    try {
                        Thread.sleep(1000);  // Limelight not ready yet: try again
                    } catch (InterruptedException stop) {
                        return;
                    }
                }
            }
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            byte[] jpeg = newest;
            Bitmap bitmap = jpeg == null ? null : BitmapFactory.decodeByteArray(jpeg, 0, jpeg.length);
            continuation.dispatch(consumer -> consumer.accept(bitmap));
        }
    }

    // ================================================================ RECOGNISING
    // Everything below only takes a Limelight picture (LLResult) and gives back plain numbers.
    // No motors, no OpMode: Auto3 and Auto4 can call these too.

    /** One blob of a color in a picture. Angles are from the middle of the picture. */
    public static class Blob {
        public double degreesRight;  // to the ROBOT's right (negative = left). The camera looks backward.
        public double degreesUp;     // above the middle of the picture (negative = below)
        public double percent;       // how much of the picture it fills
        public double inches;        // ROUGH distance from the lens, from its size (see roughInches)
    }

    /**
     * The blobs in one picture from a color slot, biggest first, without those filling less than
     * smallestPercent of the picture. ballInches (how wide the ball really is) is for the distance.
     */
    public static List<Blob> findBlobs(LLResult result, double smallestPercent, double ballInches) {
        List<Blob> blobs = new ArrayList<>();
        if (result == null || result.getColorResults() == null) {
            return blobs;
        }
        for (LLResultTypes.ColorResult found : result.getColorResults()) {
            Blob blob = new Blob();
            blob.percent = found.getTargetArea() / AREA_FULL * 100;
            if (blob.percent < smallestPercent) {
                continue;
            }
            blob.degreesRight = -found.getTargetXDegrees();  // the camera's right is the robot's left
            blob.degreesUp = BLOB_UP * found.getTargetYDegrees();
            blob.inches = roughInches(blob.percent, ballInches);
            int place = 0;
            while (place < blobs.size() && blobs.get(place).percent >= blob.percent) {
                place++;
            }
            blobs.add(place, blob);
        }
        return blobs;
    }

    /**
     * ROUGH distance (inches from the lens) of a ball that is ballInches wide and fills percent of
     * the picture: the more of the picture it fills, the closer it is. Rough because a ball that is
     * half hidden, cut off by the edge, or only partly the right color looks smaller = further away.
     */
    public static double roughInches(double percent, double ballInches) {
        // The picture is CAMERA_HFOV x CAMERA_VFOV degrees; a round blob that is d degrees wide
        // fills (pi / 4) x d x d of that.
        double degreesWide = Math.sqrt(percent / 100 * CAMERA_HFOV * CAMERA_VFOV * 4 / Math.PI);
        degreesWide = Math.max(0.01, Math.min(degreesWide, 170));  // never divide by 0
        return ballInches / 2 / Math.tan(Math.toRadians(degreesWide / 2));
    }

    /** One HIVE CELL as the camera sees it: the row of 4 AprilTags under it. Robot terms, inches. */
    public static class Cell {
        public String name;                            // "RED AUDIENCE", ...
        public int firstId;                            // the id of the first of its 4 tags
        public final boolean[] used = new boolean[4];  // which of the 4 tags the numbers come from
        public int tags;                               // how many that is
        public String notUsed = "";                    // tags that were seen but not used, and why
        public double behind;        // inches from the robot's middle back to the middle of the CELL
        public double right;         // inches the middle of the CELL is to the robot's RIGHT (negative = left)
        public double degreesRight;  // the same as an angle from straight behind
        public double aimOff;        // Auto3's aim: degrees the robot has to turn LEFT (AIM_TRIM counted in)
        public boolean auto3Target;  // Auto3 would shoot at this CELL
        public String auto3Why = ""; // why not, if not
    }

    /**
     * Collects the HIVE tags of one or more pictures (the average is steadier than one picture) and
     * works out where each CELL is. Use: clear(), add(result) for every NEW picture from the tag
     * slot, then cells().
     */
    public static class TagLook {
        // Per tag 30-45: inches behind (added up), inches right (added up), times seen. As Auto3.tagSeen.
        public final double[][] seen = new double[LAST_TAG - FIRST_TAG + 1][3];
        public final int[] noPlace = new int[LAST_TAG - FIRST_TAG + 1];  // times seen without a 3D position
        public final int[] tooFar = new int[LAST_TAG - FIRST_TAG + 1];   // times seen further than TAG_TOO_FAR
        public final TreeSet<Integer> otherTags = new TreeSet<>();       // ids that are not HIVE tags
        public int pictures;

        public void clear() {
            for (int i = 0; i < seen.length; i++) {
                seen[i][0] = seen[i][1] = seen[i][2] = 0;
                noPlace[i] = tooFar[i] = 0;
            }
            otherTags.clear();
            pictures = 0;
        }

        /** Adds the tags in one picture. The maths is the same as Auto3.readTags, keep equal. */
        public void add(LLResult result) {
            if (result == null) {
                return;
            }
            pictures++;
            if (!result.isValid() || result.getFiducialResults() == null) {
                return;  // no tags in this picture
            }
            double pitch = Math.toRadians(Auto3.CAMERA_PITCH);
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id < FIRST_TAG || id > LAST_TAG) {
                    otherTags.add(id);
                    continue;  // not a HIVE tag
                }
                // Limelight camera space: x right, y down, z straight out of the lens.
                Pose3D pose = tag.getTargetPoseCameraSpace();
                Position pos = pose == null ? null : pose.getPosition().toUnit(DistanceUnit.INCH);
                if (pos == null || pos.z <= 0) {
                    noPlace[id - FIRST_TAG]++;  // the tag is seen, but the Limelight gives no 3D position for it
                    continue;
                }
                double behind = Auto3.CAMERA_BACK + pos.z * Math.cos(pitch) + pos.y * Math.sin(pitch);
                if (behind > TAG_TOO_FAR) {
                    tooFar[id - FIRST_TAG]++;  // too far to trust
                    continue;
                }
                seen[id - FIRST_TAG][0] += behind;
                seen[id - FIRST_TAG][1] -= pos.x;  // the camera looks backward: its right is the robot's left
                seen[id - FIRST_TAG][2]++;
            }
        }

        /**
         * Every CELL with a tag in view. The maths is the same as Auto3.findCell, keep equal: a
         * CELL's 4 tags are in a row, TAG_PLACE inches from its middle; with 2 or more tags it
         * follows the line through them back to place 0, the middle of the CELL. And the CELL Auto3
         * would shoot at is the nearest one within AIM_MAX degrees of straight behind and FAR_CELL inches.
         */
        public List<Cell> cells() {
            List<Cell> cells = new ArrayList<>();
            Cell target = null;
            for (int c = 0; c < Auto3.CELLS.length; c++) {
                Cell cell = new Cell();
                cell.name = Auto3.CELLS[c];
                cell.firstId = FIRST_TAG + c * 4;
                double place = 0, behind = 0, right = 0;  // averages over the tags of this CELL that were seen
                for (int t = 0; t < 4; t++) {
                    double[] tag = seen[c * 4 + t];
                    if (tag[2] > 0) {
                        cell.used[t] = true;
                        cell.tags++;
                        place += Auto3.TAG_PLACE[t];
                        behind += tag[0] / tag[2];
                        right += tag[1] / tag[2];
                    } else if (noPlace[c * 4 + t] > 0) {
                        cell.notUsed += " " + (cell.firstId + t) + " (no 3D position)";
                    } else if (tooFar[c * 4 + t] > 0) {
                        cell.notUsed += " " + (cell.firstId + t) + String.format(" (over %.0f in.)", TAG_TOO_FAR);
                    }
                }
                if (cell.tags == 0) {
                    if (!cell.notUsed.isEmpty()) {
                        cells.add(cell);
                    }
                    continue;
                }
                place /= cell.tags;
                behind /= cell.tags;
                right /= cell.tags;
                // The line through the tags: how much further behind / right it gets per inch along the row.
                double spread = 0, behindSlope = 0, rightSlope = 0;
                for (int t = 0; t < 4; t++) {
                    double[] tag = seen[c * 4 + t];
                    if (tag[2] > 0) {
                        double along = Auto3.TAG_PLACE[t] - place;
                        spread += along * along;
                        behindSlope += along * (tag[0] / tag[2] - behind);
                        rightSlope += along * (tag[1] / tag[2] - right);
                    }
                }
                if (spread > 0) {  // 2 or more tags: back along that line to place 0, the middle of the CELL
                    behind -= behindSlope / spread * place;
                    right -= rightSlope / spread * place;
                }
                cell.behind = behind;
                cell.right = right;
                cell.degreesRight = Math.toDegrees(Math.atan2(right, behind));
                cell.aimOff = cell.degreesRight + Auto3.AIM_TRIM;
                if (Math.abs(cell.degreesRight) > Auto3.AIM_MAX) {
                    cell.auto3Why = String.format("over %.0f deg to the side", Auto3.AIM_MAX);
                } else if (behind > Auto3.FAR_CELL) {
                    cell.auto3Why = String.format("over %.0f in. away", Auto3.FAR_CELL);
                } else {
                    cell.auto3Why = "another CELL is nearer";
                    if (target == null || behind < target.behind) {
                        target = cell;
                    }
                }
                cells.add(cell);
            }
            if (target != null) {
                target.auto3Target = true;
                target.auto3Why = "";
            }
            return cells;
        }
    }
}
