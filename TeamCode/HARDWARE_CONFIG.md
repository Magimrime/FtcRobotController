# Robot hardware config (FTC 33498)

These are the device names the TeamCode OpModes look up in the hardware map. In the Driver Hub,
go to **Configure Robot**. Each name there must match exactly, lowercase.

Checked against Auto1–4, Odometry, OdometryTest, RobotHardwareTest, RPMTest and TeleOp on 2026-11-18.

| Config name | Type to pick in Configure Robot | Port | Direction in code | Used by |
|---|---|---|---|---|
| `topleft` | Motor: GoBILDA 5202/3/4 series (19.2:1, 312 RPM) | Control Hub motor 0–3 * | REVERSE | all drive OpModes |
| `topright` | Motor: GoBILDA 5202/3/4 series (19.2:1, 312 RPM) | Control Hub motor 0–3 * | FORWARD | all drive OpModes |
| `bottomleft` | Motor: GoBILDA 5202/3/4 series (19.2:1, 312 RPM) | Control Hub motor 0–3 * | REVERSE | all drive OpModes |
| `bottomright` | Motor: GoBILDA 5202/3/4 series (19.2:1, 312 RPM) | Control Hub motor 0–3 * | REVERSE | all drive OpModes |
| `mainmotorintake` | Motor: GoBILDA 5202/3/4 series (model not confirmed) | Expansion Hub motor 0 | FORWARD | TeleOp, Auto1–4, RPMTest, RobotHardwareTest |
| `turret` (flywheel) | Motor: GoBILDA 5202/3/4 series (1:1, 6000 RPM) | Expansion Hub motor 1 | FORWARD | TeleOp, Auto1–4, RPMTest, RobotHardwareTest |
| `leftfeeder` | Continuous Rotation Servo | Control Hub servo 0 | FORWARD | TeleOp, Auto1–4, RPMTest, RobotHardwareTest |
| `rightfeeder` | Continuous Rotation Servo | Expansion Hub servo 0 | REVERSE | TeleOp, Auto1–4, RPMTest, RobotHardwareTest |
| `servosiphon` | Continuous Rotation Servo | Expansion Hub servo 1 | REVERSE | TeleOp, Auto1–4, RPMTest, RobotHardwareTest |
| `imu` | REV internal IMU (BHI260AP; older hubs: BNO055) | Control Hub I2C bus 0 | hub logo UP, USB FORWARD | Auto2, Auto3, Auto4, Odometry, OdometryTest |
| `limelight` | Ethernet Device (appears after Scan) | Control Hub blue USB 3.0 port | n/a | Auto3, Auto4 |

## Notes

- **Required vs optional.** Auto4 and Odometry fail at INIT if `imu` is missing. Auto4 also fails at INIT
  if `limelight` is missing. Auto2, Auto3 and OdometryTest keep running without `imu`, and Auto3 keeps
  running without `limelight`.
- **Servos must be "Continuous Rotation Servo".** The code drives all three servos as CRServo. If one is
  set up as a regular "Servo", the OpMode fails at INIT.
- **Motor model mostly doesn't matter for the code.** The flywheel RPM is calculated from 28 ticks per
  turn in the code, not from the motor type in the config. The drive odometry uses 537.7 ticks per turn
  with 96 mm wheels.
- **Not confirmed:**
  - \* The code comments say the four wheels are on Control Hub motor ports 0–3, but they don't say
    which wheel is on which port.
  - The intake motor model isn't stated anywhere in the code.
  - The ports come from comments in TeleOp.java and RobotHardwareTest.java, not from a saved config.
    The `imu` and `limelight` ports are the usual setup, not something the code says.
- The active robot configuration is stored only on the Control Hub (`/sdcard/FIRST/*.xml`). There is
  no copy of it in this repo.
