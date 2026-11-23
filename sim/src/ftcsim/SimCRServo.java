package ftcsim;

import com.qualcomm.robotcore.hardware.CRServo;

/** A simulated continuous-rotation servo. powerSign = +1: a positive raw power turns it the useful way. */
public class SimCRServo implements CRServo {
    public final String name;
    public final int port;
    public double powerSign;
    private Direction direction = Direction.FORWARD;
    private double power = 0;

    public SimCRServo(String name, int port, double powerSign) {
        this.name = name;
        this.port = port;
        this.powerSign = powerSign;
    }

    /** -1..1 the useful way (0 after STOP). */
    public double usefulPower() {
        if (SimContext.get().robotDisabled()) return 0;
        return power * (direction == Direction.REVERSE ? -1 : 1) * powerSign;
    }

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
        if (power != this.power) {
            c.tick(c.scenario.costHubCommand);
        }
        this.power = power;
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public int getPortNumber() {
        return port;
    }

    @Override
    public String getDeviceName() {
        return "CR servo (simulated) " + name;
    }
}
