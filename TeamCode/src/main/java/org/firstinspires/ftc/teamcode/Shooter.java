package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * The shooter: a flywheel that launches balls, and the siphon servo that turns the carousel to
 * push the next ball into it.
 *
 *   turret        Expansion Hub motor port 1, encoder cable plugged in   (the flywheel)
 *   servosiphon   Expansion Hub servo port 1, continuous rotation   (turns the carousel)
 *
 * The flywheel is given a speed in RPM, not a power. The hub's built-in speed controller reads
 * the encoder and adjusts the power to hold that speed, so shots stay the same as the battery
 * drains and the wheel recovers quickly after each ball.
 *
 * The flywheel motor is a goBILDA 5203 series, 19.2:1, 312 RPM. Its encoder ticks per turn and
 * top speed are written below rather than read from the Robot Configuration, because the only
 * matching choice there, "GoBILDA 5202/3/4 series", describes the 99.5:1 version (2786 ticks,
 * 60 RPM) and would cap the target at 60 RPM. Still pick that choice for "turret", not
 * "Unspecified Motor": the hub tunes its speed controller from it, and both versions top out at
 * nearly the same encoder ticks per second, so that tuning fits. If the motor is ever swapped
 * for a different ratio, change TICKS_PER_REV and MAX_RPM to match its label.
 */
public class Shooter {

    public static final String FLYWHEEL_NAME = "turret";
    public static final String SIPHON_NAME = "servosiphon";

    // Flywheel speed, in RPM of the motor's output shaft. Tune it at the field: gamepad 2's d-pad
    // changes it by RPM_STEP while the TeleOp runs, and telemetry shows the current value.
    public static final double DEFAULT_RPM = 250.0;
    public static final double RPM_STEP = 10.0;

    // goBILDA 5203 series, 19.2:1: encoder ticks per turn of the output shaft, and its top speed
    // with nothing attached. It cannot hold a target right at the top, so expect the useful
    // range to end somewhere around 280.
    public static final double TICKS_PER_REV = 537.7;
    public static final double MAX_RPM = 312.0;

    // The flywheel is ready to shoot once its speed is within this fraction of the target. Each
    // shot slows it briefly, so firing pauses until it has recovered.
    public static final double READY_TOLERANCE = 0.05;

    // Siphon servo power, 0 to 1.
    public static final double SIPHON_POWER = 1.0;

    private final DcMotorEx flywheel;
    private final CRServo siphon;
    private final MotorConfigurationType motorType;
    private final double ticksPerRev;
    private final double maxRpm;

    private double targetRpm;
    private boolean flywheelOn = false;
    private double siphonPower = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        siphon = hardwareMap.get(CRServo.class, SIPHON_NAME);

        // Positive flywheel speed must launch balls OUT of the shooter, and positive siphon power
        // must move balls toward the flywheel. Check both with the "Robot Hardware Test" utility
        // and flip FORWARD/REVERSE on anything that runs the wrong way.
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // REVERSE is goBILDA's windmill direction, carried over from the TeleOp that used to run
        // this servo directly and was checked on the robot.
        siphon.setDirection(DcMotorSimple.Direction.REVERSE);

        // FLOAT lets the flywheel coast to a stop instead of braking hard, which is easier on the
        // gears. The run mode is switched in startFlywheel() and stopFlywheel(); see there.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The configured motor model is kept only to show on telemetry; see the top of this file
        // for why the ticks per turn and top speed are not taken from it.
        motorType = flywheel.getMotorType();
        ticksPerRev = TICKS_PER_REV;
        maxRpm = MAX_RPM;
        targetRpm = Range.clip(DEFAULT_RPM, 0.0, maxRpm);
    }

    /** Spins the flywheel up to the target speed and holds it there. */
    public void startFlywheel() {
        if (!flywheelOn) {
            flywheelOn = true;
            // The speed controller only works in RUN_USING_ENCODER mode.
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        flywheel.setVelocity(rpmToTicksPerSecond(targetRpm));
    }

    /** Lets the flywheel coast to a stop. */
    public void stopFlywheel() {
        flywheelOn = false;
        // In RUN_USING_ENCODER mode, zero power means "hold a speed of zero", which brakes the
        // flywheel hard. Leaving that mode first makes zero power really zero, so it coasts.
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(0.0);
    }

    public void toggleFlywheel() {
        if (flywheelOn) {
            stopFlywheel();
        } else {
            startFlywheel();
        }
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    /** Changes the target speed by delta RPM (negative slows it). Takes effect immediately. */
    public void adjustTargetRpm(double delta) {
        setTargetRpm(targetRpm + delta);
    }

    /** Sets the target speed in RPM, limited to what the motor can do. */
    public void setTargetRpm(double rpm) {
        targetRpm = Range.clip(rpm, 0.0, maxRpm);
        if (flywheelOn) {
            flywheel.setVelocity(rpmToTicksPerSecond(targetRpm));
        }
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** The flywheel's measured speed in RPM, from the encoder. */
    public double getRpm() {
        return flywheel.getVelocity() * 60.0 / ticksPerRev;
    }

    /** True when the flywheel is on and close enough to the target speed to shoot. */
    public boolean isReady() {
        return isReady(getRpm());
    }

    private boolean isReady(double rpm) {
        return flywheelOn && targetRpm > 0.0
                && Math.abs(rpm - targetRpm) <= READY_TOLERANCE * targetRpm;
    }

    /** Turns the carousel forward, pushing the next ball into the flywheel. */
    public void feed() {
        runSiphon(SIPHON_POWER);
    }

    /** Turns the carousel backward, to clear a jam. */
    public void feedReverse() {
        runSiphon(-SIPHON_POWER);
    }

    public void stopFeeding() {
        runSiphon(0.0);
    }

    /** Runs the siphon servo directly: -1 to 1, positive moves balls toward the flywheel. */
    public void runSiphon(double power) {
        siphon.setPower(power);
        siphonPower = power;
    }

    public boolean isFeeding() {
        return siphonPower > 0.0;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm / 60.0 * ticksPerRev;
    }

    /** Adds the configured ports and flywheel motor model, to compare with the top of this file. */
    public void addPortTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter ports", "flywheel %d   siphon %d",
                flywheel.getPortNumber(), siphon.getPortNumber());
        telemetry.addData("Flywheel motor", "%.1f ticks/turn, %.0f RPM max   (configured as %s)",
                ticksPerRev, maxRpm, motorType.getName());
    }

    /** Adds the shooter state to telemetry. The caller still calls telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        double rpm = getRpm();
        String state = !flywheelOn ? "OFF" : isReady(rpm) ? "READY" : "SPINNING UP";
        telemetry.addData("Shooter", "%s   %.0f / %.0f rpm", state, rpm, targetRpm);
        telemetry.addData("Siphon", siphonPower > 0.0 ? "Feeding" : siphonPower < 0.0 ? "Reverse" : "Stopped");
        if (flywheelOn && rpm == 0.0) {
            telemetry.addLine("Flywheel reads 0 rpm: check its encoder cable");
        }
    }
}
