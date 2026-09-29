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
 *   turret        Expansion Hub motor port 0, encoder cable plugged in   (the flywheel)
 *   servosiphon   servo port 3, continuous rotation                      (turns the carousel)
 *
 * The flywheel is given a speed in RPM, not a power. The hub's built-in speed controller reads
 * the encoder and adjusts the power to hold that speed, so shots stay the same as the battery
 * drains and the wheel recovers quickly after each ball. This only works if the "turret" motor
 * in the Robot Configuration is set to the real motor model, not "Unspecified Motor": the
 * controller's tuning and the RPM figures both come from the model's encoder ticks per turn and
 * top speed. Telemetry shows the model the robot is using.
 */
public class Shooter {

    public static final String FLYWHEEL_NAME = "turret";
    public static final String SIPHON_NAME = "servosiphon";

    // Flywheel speed, in RPM of the motor's output shaft. Tune it at the field: gamepad 2's d-pad
    // changes it by RPM_STEP while the TeleOp runs, and telemetry shows the current value.
    public static final double DEFAULT_RPM = 3000.0;
    public static final double RPM_STEP = 100.0;

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
        siphon.setDirection(DcMotorSimple.Direction.FORWARD);

        // FLOAT lets the flywheel coast to a stop instead of braking hard, which is easier on the
        // gears. The run mode is switched in startFlywheel() and stopFlywheel(); see there.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Encoder ticks per turn and top speed come from the motor model in the configuration.
        motorType = flywheel.getMotorType();
        ticksPerRev = motorType.getTicksPerRev();
        maxRpm = motorType.getMaxRPM();
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

    /** Sets the target speed in RPM, limited to what the motor model can do. */
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
        telemetry.addData("Flywheel motor model", "%s: %.0f ticks/turn, %.0f RPM max",
                motorType.getName(), ticksPerRev, maxRpm);
    }

    /** Adds the shooter state to telemetry. The caller still calls telemetry.update(). */
    public void addTelemetry(Telemetry telemetry) {
        double rpm = getRpm();
        String state = !flywheelOn ? "OFF" : isReady(rpm) ? "READY" : "SPINNING UP";
        telemetry.addData("Shooter", "%s   %.0f / %.0f rpm", state, rpm, targetRpm);
        telemetry.addData("Siphon", siphonPower > 0.0 ? "Feeding" : siphonPower < 0.0 ? "Reverse" : "Stopped");
        if (flywheelOn && rpm == 0.0) {
            telemetry.addLine("Flywheel reads 0 rpm: check its encoder cable and motor model");
        }
    }
}
