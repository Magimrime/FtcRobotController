package ftcsim;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Random;

/**
 * The Control Hub's BHI260AP IMU. Yaw follows the robot's TRUE heading since resetYaw(), with the
 * scenario's scale error, bias drift, random walk and per-sample noise. New samples every 2.5 ms;
 * reading twice inside one sample gives the same numbers, as on the real hub.
 */
public class SimIMU implements IMU {
    private final World world;
    private final Scenario s;
    private final Random rng;
    private double headingAtReset = 0, resetTime = 0, walk = 0;
    private double lastSampleTime = -1;
    private double yaw, pitch, roll;
    private boolean frozen = false;

    public SimIMU(World world, Scenario s, Random rng) {
        this.world = world;
        this.s = s;
        this.rng = rng;
    }

    @Override
    public boolean initialize(Parameters parameters) {
        SimContext.get().tick(0.1);
        resetYaw();
        return true;
    }

    @Override
    public void resetYaw() {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHubCommand);
        headingAtReset = world.trueHeadingDeg();
        resetTime = c.now();
        walk = 0;
        lastSampleTime = -1;
    }

    private void sample() {
        SimContext c = SimContext.get();
        double t = c.now();
        if (frozen) return;
        if (s.imuFreezeAt >= 0 && c.isStarted() && c.matchTime() >= s.imuFreezeAt) {
            frozen = true;
            return;
        }
        if (lastSampleTime >= 0 && t - lastSampleTime < 0.0025) return;
        double dt = lastSampleTime < 0 ? 0 : t - lastSampleTime;
        lastSampleTime = t;
        walk += rng.nextGaussian() * s.imuWalk * Math.sqrt(dt);
        double turned = world.trueHeadingDeg() - headingAtReset;
        yaw = Field.wrapDeg(turned * (1 + s.imuScale) + s.imuBias * (t - resetTime) + walk + rng.nextGaussian() * s.imuNoise);
        pitch = rng.nextGaussian() * Math.max(s.imuNoise, 0.01) * 2;
        roll = rng.nextGaussian() * Math.max(s.imuNoise, 0.01) * 2;
    }

    @Override
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costImuRead);
        sample();
        return new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, c.nanos());
    }

    @Override
    public String getDeviceName() {
        return "BHI260AP IMU (simulated)";
    }
}
