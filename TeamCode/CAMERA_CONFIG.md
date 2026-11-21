# Camera config (FTC 33498)

Everything the code sets up on the camera, and what each setting is good for.
Checked against Auto3, Auto4, CameraDetection and TeleOp on 2026-11-21.

## The camera

| What | Value | Where it is in the code |
|---|---|---|
| Camera | Limelight 3A | |
| Config name | `limelight` (Ethernet Device, appears after Scan) | `CAMERA_NAME` in Auto3, Auto4, TeleOp |
| Plugged into | Control Hub blue USB 3.0 port (also powers it) | |
| Mounted | On the BACK, next to the launcher, looking the way the launcher shoots. It can't turn. | |
| Behind the robot's middle | 8 in. (guess, measure it) | `CAMERA_BACK` in Auto3 and Auto4 |
| Height above the floor | 8 in. (guess, measure it) | `CAMERA_HEIGHT` in Auto4 |
| Tilted up | 15 deg (guess, measure it) | `CAMERA_PITCH` in Auto3 and Auto4 |
| Angle of view | 54.5 deg wide, 42 deg high | `CAMERA_HFOV`, `CAMERA_VFOV` in Auto4 and CameraDetection |
| Picture | 640 x 480 | not set by the code |

Because the camera looks backward, everything the code shows on the Driver Hub is already turned into
the ROBOT's left and right.

## The slots (Limelight "pipelines")

The Limelight can only look for one thing at a time. Each thing has its own numbered slot, and the
settings of every slot are stored on the camera itself, so they stay after a power cycle. The code
writes them at INIT. The Limelight web page can change them too, but the code writes its own numbers
again at the next INIT.

| Slot | Type | Looks for | Set up by | Used by |
|---|---|---|---|---|
| 0 | color (`pipe_color`) | nothing: it is the plain color picture | Auto4 | Auto4 finds the balls in this picture itself. TeleOp shows it on the Camera Stream. |
| 1 | AprilTags (`pipe_fiducial`) | the HIVE tags | Auto3, Auto4, CameraDetection (the same numbers) | Auto3 aim check, Auto4 aiming, CameraDetection TAGS look |
| 2 | color | YELLOW POLLEN | CameraDetection | CameraDetection YELLOW look |
| 3 | color | RED NECTAR | CameraDetection | CameraDetection RED look |
| 4 | color | BLUE NECTAR | CameraDetection | CameraDetection BLUE look |
| 5 to 9 | | free | | |

The picture of an AprilTag slot is always black and white. Only color slots give a color picture.

### Slot 1: AprilTags

| Setting | Value | Constant | What it does |
|---|---|---|---|
| Tag size | 82.55 mm (3.25 in.) | `fiducial_size` in setUp | Must match the real tags, or every distance is wrong. |
| Exposure | 1500 | `TAG_EXPOSURE` (Auto3), `START_EXPOSURE` (Auto4) | How long each picture is lit. Higher = brighter, but blurrier when the robot moves. |
| Gain | 30 | `TAG_GAIN` (Auto3), `GAIN` (Auto4) | Brightens the picture electronically. Higher = brighter but grainier. Grain makes tags harder to find. |
| Detector downscale | 1 | `TAG_DOWNSCALE` (Auto3, Auto4) | See below. |

**Detector downscale** is how much the camera shrinks the picture before it searches it for tags.

| Downscale | Range | Pictures a second (measured on our robot) | Good for |
|---|---|---|---|
| 1 (now) | longest: about twice as far, or as sideways, as 2 | about 45 | Standing still and aiming at a CELL. This is what all our OpModes do. |
| 2 (what slot 1 had before) | about half of 1 | about 56 | Reading tags while driving fast, where more pictures a second matter more than range. |
| 3, 4 | shorter still | more | Not useful for us. |

With downscale 2, the CELL tags at about 64 in. were only about 34 pixels wide and some were missed
(2026-11-21). With 1, both a red and a blue CELL were found in the same picture.

**The HIVE tags** (from the SDK's BIOBUZZ tag list): family 36h11, 3.25 in., 4 in a row under each CELL.

| CELL | Tag IDs | Who shoots at it |
|---|---|---|
| RED SCORING | 30, 31, 32, 33 | |
| RED AUDIENCE | 34, 35, 36, 37 | red alliance (start against the audience wall) |
| BLUE AUDIENCE | 38, 39, 40, 41 | |
| BLUE SCORING | 42, 43, 44, 45 | blue alliance (start against the back wall) |

No tag filter is set on the camera, on purpose. The code takes the CELL that is straight behind the
robot (within `AIM_MAX` 10 deg and `FAR_CELL` 65 in. in Auto3), so other CELLS in the picture don't
confuse it.

### Slot 0: color picture (Auto4, TeleOp)

| Setting | Value | Constant |
|---|---|---|
| Exposure | starts at 1500, Auto4 raises it in INIT up to 3300 when the picture is too dark | `START_EXPOSURE`, `MAX_EXPOSURE` (Auto4) |
| Gain | 30 | `GAIN` (Auto4) |
| Black level | 0 | in Auto4 setUp |
| Picture shown | the camera picture, not a threshold picture | `img_to_show` 0 in Auto4 setUp |

### Slots 2 to 4: ball colors (CameraDetection)

From the color table `COLORS` at the top of CameraDetection.java. Exposure and gain are
`COLOR_EXPOSURE` and `COLOR_GAIN`, which start equal to the tag slot's 1500 and 30.

| Slot | Ball | Hue | Saturation | Value (brightness) | Ball size |
|---|---|---|---|---|---|
| 2 | YELLOW POLLEN | 20 to 35 | 100 to 255 | 100 to 255 | 2.8 in. |
| 3 | RED NECTAR | outside 10 to 170 (red is at both ends) | 120 to 255 | 80 to 255 | 3.6 in. |
| 4 | BLUE NECTAR | 100 to 125 | 100 to 255 | 60 to 255 | 3.6 in. |

These are starting guesses. How to tune them is at the top of CameraDetection.java.

## Which OpMode for what

| I want to... | Use |
|---|---|
| See what the camera sees before a match | TeleOp, INIT, then menu > Camera Stream (slot 0, color) |
| Check which HIVE tags the camera finds | Camera Detection: INIT, dpad right once ("Look: TAGS only"), then Camera Stream. Found tags get a green box with their number. |
| Tune a ball color | Camera Detection: dpad to that color, then the Limelight web page |
| See the camera's stored settings | Camera Detection: dpad to the look, then Y |
| Check the aim before a match | Auto3 INIT: the "Camera" line says the CELL, distance and whether it will turn |

**Reading the Camera Stream:** it is one picture per tap. The two small green numbers in the top left
corner are pictures a second and, under it, the slot the picture came from (1 = tags). On Camera
Detection's ALL look, most pictures come from a color slot and show no tag boxes, so pick one look first.
The gamepad doesn't work while the Camera Stream is open.

**The Limelight web page:** plug the camera's USB cable into a laptop instead of the Control Hub and open
http://limelight.local:5801 (or http://172.29.0.1:5801). Plug it back into the Control Hub afterwards.

## Things to try if tags are still missed

1. **More light on the tags.** The pictures are grainy. Better light lets you lower the gain.
2. **Lower gain, higher exposure** (for example gain 15, exposure 3000) for a cleaner picture. Fine while
   the robot stands still, blurry while it moves.
3. **Bigger picture (1280 x 960)** roughly doubles the range again, but gives only about 20 pictures a
   second. The code doesn't set the picture size yet. After changing it, check the distance on the
   Driver Hub against a tape measure.

## Not confirmed yet

- The names the code uses for camera settings (`fiducial_resdiv`, `img_to_show`, `hue_min` ...) are
  best guesses. The SETUP lines of Camera Detection say whether the camera took each one. The drop from
  about 56 to 45 pictures a second suggests `fiducial_resdiv` works.
- `CAMERA_BACK`, `CAMERA_HEIGHT`, `CAMERA_PITCH` are guesses. Measure them.
- `AREA_FULL` and `BLOB_UP` in CameraDetection (ball size unit and up/down sign) are not checked.
- The 640 x 480 picture size is the Limelight 3A's normal setting, not something the code sets or reads.
