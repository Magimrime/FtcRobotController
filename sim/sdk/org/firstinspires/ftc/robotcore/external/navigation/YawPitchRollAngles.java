package org.firstinspires.ftc.robotcore.external.navigation;

/** SIMULATOR copy of the FTC SDK class. */
public class YawPitchRollAngles {
    private final AngleUnit unit;
    private final double yaw, pitch, roll;
    private final long acquisitionTime;

    public YawPitchRollAngles(AngleUnit unit, double yaw, double pitch, double roll, long acquisitionTime) {
        this.unit = unit;
        this.yaw = yaw;
        this.pitch = pitch;
        this.roll = roll;
        this.acquisitionTime = acquisitionTime;
    }

    public double getYaw() {
        return unit.toDegrees(yaw);
    }

    public double getYaw(AngleUnit angleUnit) {
        return angleUnit.fromUnit(unit, yaw);
    }

    public double getPitch() {
        return unit.toDegrees(pitch);
    }

    public double getPitch(AngleUnit angleUnit) {
        return angleUnit.fromUnit(unit, pitch);
    }

    public double getRoll() {
        return unit.toDegrees(roll);
    }

    public double getRoll(AngleUnit angleUnit) {
        return angleUnit.fromUnit(unit, roll);
    }

    public long getAcquisitionTime() {
        return acquisitionTime;
    }

    @Override
    public String toString() {
        return String.format("{yaw=%.3f, pitch=%.3f, roll=%.3f}", getYaw(), getPitch(), getRoll());
    }
}
