package ftcsim;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * A simulated goBILDA motor on a REV hub. Like the SDK's DcMotorImpl, setDirection(REVERSE)
 * flips both the power sent and the encoder count read. Under that, the HARDWARE TRUTH:
 *   powerSign:   +1 = a positive raw power moves the mechanism its useful way (wheel forward, intake in)
 *   encoderSign: +1 = the raw encoder counts up when the mechanism moves its useful way
 * The code's directions are right when powerSign x direction = +1; the BR wheel with swapped power
 * wires has the right power but a backward encoder.
 */
public class SimMotor implements DcMotorEx {
    public final String name;
    public final LynxModule hub;
    public final int port;
    public final double ticksPerRev;
    public double powerSign, encoderSign;
    public boolean encoderDead = false;

    private Direction direction = Direction.FORWARD;
    private ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.FLOAT;
    private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    private double power = 0;
    private double velocityTarget = 0;   // ticks/s, code's sign
    private boolean velocityControl = false;
    private int targetPosition = 0;
    private boolean enabled = true;

    // what the world says the mechanism did, in raw encoder ticks (useful direction = +)
    private double usefulTicks = 0, usefulTicksPerSec = 0;
    private double resetOffset = 0;
    private double cachedRawTicks = 0, cachedRawVel = 0;

    public SimMotor(String name, LynxModule hub, int port, double ticksPerRev, double powerSign, double encoderSign) {
        this.name = name;
        this.hub = hub;
        this.port = port;
        this.ticksPerRev = ticksPerRev;
        this.powerSign = powerSign;
        this.encoderSign = encoderSign;
        hub.simRegister(this, this::snapshot);
    }

    private double dirSign() {
        return direction == Direction.REVERSE ? -1 : 1;
    }

    // ------------------------------------------------------------ for the world

    /** Power moving the mechanism its useful way, -1..1 (0 after STOP). */
    public double usefulPower() {
        SimContext c = SimContext.get();
        if (c.robotDisabled() || !enabled) return 0;
        return power * dirSign() * powerSign;
    }

    /** True when the code drives this motor by velocity (RUN_USING_ENCODER + setVelocity). */
    public boolean velocityControlled() {
        return velocityControl && mode == RunMode.RUN_USING_ENCODER;
    }

    /** Velocity target in useful-direction ticks/s. */
    public double usefulVelocityTarget() {
        SimContext c = SimContext.get();
        if (c.robotDisabled() || !enabled) return 0;
        return velocityTarget * dirSign() * powerSign;
    }

    public ZeroPowerBehavior zeroPower() {
        return zeroPowerBehavior;
    }

    /** The world moved the mechanism to this position / speed (useful-direction ticks). */
    public void setTruth(double usefulTicks, double usefulTicksPerSec) {
        this.usefulTicks = usefulTicks;
        this.usefulTicksPerSec = usefulTicksPerSec;
    }

    /** The hub read this motor's encoder now (see LynxModule's bulk caching). */
    public void snapshot() {
        double s = encoderDead ? 0 : encoderSign;
        cachedRawTicks = usefulTicks * s - resetOffset;
        cachedRawVel = usefulTicksPerSec * s;
    }

    // ------------------------------------------------------------ DcMotor

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public void setPower(double power) {
        SimContext c = SimContext.get();
        power = Math.max(-1, Math.min(1, power));
        if (power != this.power || velocityControl) {
            c.tick(c.scenario.costHubCommand);
        }
        this.power = power;
        velocityControl = false;
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHubCommand);
        this.zeroPowerBehavior = zeroPowerBehavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    public void setMode(RunMode mode) {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHubCommand);
        if (mode == RunMode.STOP_AND_RESET_ENCODER) {
            snapshot();
            resetOffset += cachedRawTicks;
            power = 0;
            velocityControl = false;
            return;
        }
        this.mode = mode;
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    @Override
    public int getCurrentPosition() {
        hub.simRead(this);
        return (int) Math.round(cachedRawTicks * dirSign());
    }

    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return false;
    }

    @Override
    public int getPortNumber() {
        return port;
    }

    // ------------------------------------------------------------ DcMotorEx

    @Override
    public void setMotorEnable() {
        enabled = true;
    }

    @Override
    public void setMotorDisable() {
        enabled = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return enabled;
    }

    @Override
    public void setVelocity(double angularRate) {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHubCommand);
        velocityTarget = angularRate;
        velocityControl = true;
        power = Math.max(-1, Math.min(1, angularRate / (6000 * ticksPerRev / 60.0)));
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocity(unit.toRadians(angularRate) / (2 * Math.PI) * ticksPerRev);
    }

    @Override
    public double getVelocity() {
        hub.simRead(this);
        return cachedRawVel * dirSign();
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return unit.fromRadians(getVelocity() / ticksPerRev * 2 * Math.PI);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHubCommand);
    }

    @Override
    public void setPositionPIDFCoefficients(double p) {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHubCommand);
    }

    // ------------------------------------------------------------ current (set by the world every step)

    private double amps = 0, alertAmps = 5;

    /** The world: this motor draws this much current now. */
    public void setAmps(double amps) {
        this.amps = amps;
    }

    @Override
    public double getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit unit) {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHubCommand);  // its own hub command
        return unit.convert(amps, org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
    }

    @Override
    public double getCurrentAlert(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit unit) {
        return unit.convert(alertAmps, org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
    }

    @Override
    public void setCurrentAlert(double current, org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit unit) {
        alertAmps = org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS.convert(current, unit);
    }

    @Override
    public boolean isOverCurrent() {
        return amps > alertAmps;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
    }

    @Override
    public int getTargetPositionTolerance() {
        return 5;
    }

    @Override
    public String getDeviceName() {
        return "goBILDA motor (simulated) " + name;
    }
}
