# BIOBUZZ AUTO simulator

Runs the **real, unchanged** `Auto4.java` and `Auto3.java` from `TeamCode/` against a simulated
robot, camera and field, so an AUTO can be tried in many setups on the PC in seconds. No robot, no
Android Studio, no Gradle: only a JDK (`javac` / `java` on PATH; Java 25 is installed here).

How it works: `sim/sdk/` is a stand-in for the parts of the FTC SDK the OpModes use (LinearOpMode,
HardwareMap, DcMotor/DcMotorEx, CRServo, IMU, LynxModule, Limelight3A + LLResult, ElapsedTime,
Telemetry, Gamepad, org.json). The OpMode source files are compiled straight from TeamCode against
it, so the robot code is exactly what goes on the robot. Behind those fakes, `sim/src/ftcsim/` runs
the physics and the match. The clock is simulated: it only moves when the code does something
that takes time on a Control Hub (hub reads and writes, IMU reads, telemetry, `sleep`), so a 30 s
AUTO takes well under a second and every run with the same seed is identical.

## Run it

From the repo folder in PowerShell:

```powershell
.\sim\run-sim.ps1
```
The full matrix (115 matches, about 15 s). The report is `sim\results\all-<time>\summary.md`.

```powershell
.\sim\run-sim.ps1 matrix suite=quick
```
16 Auto4 matches (both alliances, both start spots, both CELLs up; noise-free and typical).

```powershell
.\sim\run-sim.ps1 run alliance=blue start=far up=far noise=harsh seed=3
```
One match. It prints the result and the timeline, and writes `sim\results\single\<name>.log/.csv/.svg`.

```powershell
.\sim\run-sim.ps1 run opmode=Auto3 path=red up=far
```

Suites: `all` (default), `auto4`, `auto3`, `stress`, `misses` (balls that don't go in, a HIVE that
doesn't tip), `quick`. `seeds=5` runs more random robots per
setup. Any setting below can be added to `run` or `matrix` as `name=value` and then applies to
every match (e.g. `matrix suite=auto4 strafeEff=0.8`).

## Auto5 (webcam)

Auto5 runs unchanged too: `sim/sdk` also fakes VisionPortal, the AprilTag processor (single tags,
`getFreshDetections()` null until a new picture), exposure / gain controls, `WebcamName` and
`AppUtil.ROBOT_DATA_DIR`. The C270 is modelled with its narrower view (45 x 35 degrees), about 22
results a second at decimation 2, 1.5 s to start streaming, and a slower Control Hub loop while the tag
finder runs. Auto5 writes its own run log as on the robot; the simulator keeps it as
`runs/<name>.robotlog.csv` (same columns as `/sdcard/FIRST/data/auto5-log-N.csv`, so the same plotting
works). Auto5 matches run one at a time for that, so a full matrix takes about 40 s.

## HIVE geometry (`hive=`)

`practice` (default): what Auto5's run logs measured on Jin's practice HIVE, tags of an up CELL about
29.5 in. high and of a down CELL about 8.5 in., faces tilting 45 degrees with the HIVE. `guess`: the
first simulator's guess, 34 and 28 in., 17.5 degrees. Auto5's ONE_CELL_SPLIT = 19 only makes sense
on the practice numbers.

## What a match covers

* **Driver Station**: INIT (the code's camera setup and INIT loop run), the driver's INIT buttons
  for the picks (Auto4: B/Y alliance, dpad start spot, X up-CELL pick; Auto3: X path), START,
  and STOP at 30 s, after which motors are cut and the OpMode must end within 2 s.
* **Drive**: mecanum wheels with motor lag, battery voltage, per-wheel strength and diameter error,
  sideways roller slip (`strafeEff`), turning scrub, and loss of grip on hard starts and stops,
  when the wheels spin or skid and the encoders count inches the robot never moved. Walls and the
  4 HIVE frame legs stop the robot (the wheels keep turning); pushing a wall at an angle squares
  the robot to it. The back-right encoder counts backward, as the swapped power wires suggest.
* **Control Hub timing**: bulk-cached encoder reads per hub (2 hubs, as in `biobuzz.xml`),
  motor writes, IMU reads and telemetry each cost time, so a loop takes about 10-12 ms.
* **IMU**: true heading since `resetYaw()`, with scale error, bias drift, random walk, noise, and
  an optional freeze.
* **Limelight 3A**: 45 pictures/s (55 at downscale 2), 30 ms latency, 54.5 x 42 degree view. Each
  of the 16 HIVE tags is projected from the camera's TRUE mount, which can differ from the
  code's `CAMERA_BACK/HEIGHT/PITCH` guesses. A tag is found only if it is in the picture, faces
  the camera, is big enough in pixels and not too steep. Its 3D pose gets distance-dependent noise,
  a per-run scale error, occasional bad poses (misreads), whole-run occlusion by the frame, and
  dropouts. Pipelines, `pipelineSwitch`, `getStatus().getPipelineType()` and the private
  `updatePipeline` / `getPipelineAtIndex` that Auto3/Auto4 reach by reflection all work.
* **HIVEs**: each alliance's HIVE is a seesaw. Tags hang under the CELLs at `upH`/`downH` and
  their faces tilt with the HIVE. Balls into the up CELL count; `tipBalls` of them tip it after
  `tipDelay`. Another option makes the partner tip our HIVE.
* **Launcher**: flywheel spin-up and dip per ball (voltage-dependent); the siphon feeds one ball
  per quarter turn at its TRUE speed (`siphonSecPerTurn`). The range follows RPM squared (53.5 in.
  at 2800 RPM), with spread and bounce-outs.
* **GARDEN**: 4 POLLEN against the near wall; the intake takes one when it passes under the front.
* **Rules and points**: G402 midline (any corner past the middle in AUTO), HIVE frame contact,
  volley at a DOWN CELL; TIPs x 20, LEAVE 3 (not touching a wall at the end), PARK 5 (any part
  over the LOADING ZONE).

**PASS** = no crash, no midline crossing, no HIVE leg contact, no volley started at a DOWN CELL,
and at least one TIP. Points, LEAVE, PARK, odometry error and the closest approach to the
midline and to a leg are reported for every run.

## Output files

* `viewer.html`: the **replay page**. Double-click it (any browser, works offline). Pick a match
  from the PASS/FAIL chips at the bottom and press play: the robot drives on the field as the
  audience sees it, with its true path, where its own odometry thought it was (dashed ghost), the
  camera's view (darker when it sees tags), CELLs flipping UP/down, POLLEN in the air and where each
  ball landed, red rings on a midline crossing or HIVE leg touch, live readouts (step, POLLEN held,
  flywheel, battery, tags, odometry error), the odometry error over time, and the timeline of what
  happened (click a line to jump there). Space = play/pause, arrow keys = 1 s back/forward.
  `run` writes the same page for one match as `single\<name>.html`. The page template is
  `sim/viewer/viewer.html`; it loads `viewer-data.json` when it has no matches inside it.
* `summary.md`: one row per setup (seeds grouped), then one row per run with links.
* `runs/<name>.log`: result, every volley (true pose, aim error, where each ball went), rule
  events, a timeline of steps and the notes the OpMode put on its screen, the Driver Station
  screen at START and at the end, and every setting.
* `runs/<name>.svg`: the field as the audience sees it, with the true path (green) and the
  robot's own odometry (purple dashed), the volleys, and the start and end poses.
* `runs/<name>.csv`: every 50 ms: true pose, odometry, slipping, POLLEN held, flywheel RPM, volts, step.

## Noise presets (`noise=`)

| | none | typical | harsh |
|---|---|---|---|
| strafe efficiency (code assumes 0.9) | 0.90 | 0.82-0.90 | 0.75-0.85 |
| grip (in/s^2 fwd / side), wheel slip | no slip | 230 / 170, 0.25 | 170 / 120, 0.4 |
| start placement error (1 sigma) | 0 | 0.5 in., 1 deg | 1.5 in., 2.5 deg |
| IMU drift / scale | 0 | up to 0.03 deg/s, 0.3 % | up to 0.08 deg/s, 0.8 % |
| battery at START | 12.6 V | 12.4-13.0 V, sags | 11.9-12.3 V, sags more |
| camera mount vs code (pitch) | exact | +/-1.5 deg | +/-3 deg |
| camera noise, misreads, dropouts | tiny | 0.4 px, 1 %, 1 per 50 s | 0.8 px, 4 %, 1 per 12 s |
| siphon seconds per turn (code 1.5) | 1.5 | 1.35-1.65 | 1.25-1.8 |

The random values come from `seed`; `name=value` overrides always win.

## Settings (most used)

`opmode` Auto4/Auto3 · `alliance` red/blue · `start` near/far · `up` near/far (the CELL really up
at START) · `pick` camera/near/far (Auto4's X pick) · `path` green/red (Auto3) · `noise` · `seed`
`startErrX startErrY startErrH` · `strafeEff kSlip aMaxF aMaxS wheelGain diamErr deadEncoder brEncoderBackward`
`imuBias imuScale imuNoise imuFreezeAt` · `batteryV0 batteryR batteryDrain` · `siphonSecPerTurn`
`camera camNoFrames camBack camHeight camPitch camYaw camDropRate camDropLen camMisread camOcclude camDownscale camPipe1Type`
`upH downH tagTilt tipBalls tipDelay partnerTip shotMiss` · `bumpAt bumpX bumpY bumpH`
`missBalls=1,2` (those balls, counted from the first shot of the match, bounce out of the CELL) ·
`noTip=1` (those volleys go in but the HIVE does not tip) · `shotMiss=0.3` (random chance each ball bounces out)
`missPickups=1,2` (the intake fails to grab those GARDEN POLLEN, 1 = nearest our alliance wall) ·
`intakeMiss=0.25` (random chance per GARDEN ball) · `gardenCount=2` (only that many POLLEN in the GARDEN)
`hiveLegs legHalf lzX0 lzX1 lzY0 lzY1` · `config=TeamCode/hardware-config/biobuzz.xml` (take the device
names from a robot configuration file). The complete list with comments is `sim/src/ftcsim/Scenario.java`.

## Guesses that change results (measure on the real field)

* HIVE geometry: tag heights 34 in. (up) and 28 in. (down), tags tilting with the HIVE (17 degrees
  from those heights; `tagTilt=0` = always facing straight down), 3 POLLEN to tip, CELL opening 12 x 9 in.
* HIVE frame legs: 2 in. square posts at the corners of the 49.46 x 38.95 in. frame.
* LOADING ZONE: X 0-24, Y 120-144 "our side". GARDEN POLLEN: X 3.5-14 along the near wall.
* Robot: 17 x 17 in., wheels 350 x 300 mm apart, 54 in/s top speed, launcher range 53.5 in. at 2800 RPM.

## Not simulated

Other robots (except a knock and the partner tipping our HIVE), balls lying on the floor after a
TIP, FLOWERS, the color pipelines (the camera only reports AprilTags), tag orientation in
`getTargetPoseCameraSpace()` (position only), and an IMU mounted differently from HUB_LOGO / HUB_USB.

## Adding an OpMode

Add its class name to `$opmodes` in `run-sim.ps1`, and its INIT button presses to
`SimContext.planPresses()`. If its OpMode uses an SDK call the fakes do not have yet, the compile
error names it; add that method to the matching file in `sim/sdk`.
