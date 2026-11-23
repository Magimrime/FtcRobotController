package org.firstinspires.ftc.robotcore.external.navigation;

/** SIMULATOR copy of the FTC SDK enum (same behavior for what the team uses). */
public enum DistanceUnit {
    METER(1.0), CM(0.01), MM(0.001), INCH(0.0254);

    private final double meters;

    DistanceUnit(double meters) {
        this.meters = meters;
    }

    public double fromMeters(double m) {
        return m / meters;
    }

    public double fromInches(double in) {
        return in * 0.0254 / meters;
    }

    public double fromUnit(DistanceUnit them, double theirs) {
        return theirs * them.meters / meters;
    }

    public double toMeters(double inOurUnits) {
        return inOurUnits * meters;
    }

    public double toInches(double inOurUnits) {
        return inOurUnits * meters / 0.0254;
    }

    public double toMm(double inOurUnits) {
        return inOurUnits * meters / 0.001;
    }

    public double toCm(double inOurUnits) {
        return inOurUnits * meters / 0.01;
    }
}
