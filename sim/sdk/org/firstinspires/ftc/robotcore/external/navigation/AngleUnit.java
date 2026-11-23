package org.firstinspires.ftc.robotcore.external.navigation;

/** SIMULATOR copy of the FTC SDK enum (same behavior for what the team uses). */
public enum AngleUnit {
    DEGREES, RADIANS;

    public double fromDegrees(double degrees) {
        return this == DEGREES ? degrees : Math.toRadians(degrees);
    }

    public double fromRadians(double radians) {
        return this == RADIANS ? radians : Math.toDegrees(radians);
    }

    public double fromUnit(AngleUnit them, double theirs) {
        return them == DEGREES ? fromDegrees(theirs) : fromRadians(theirs);
    }

    public double toDegrees(double inOurUnits) {
        return this == DEGREES ? inOurUnits : Math.toDegrees(inOurUnits);
    }

    public double toRadians(double inOurUnits) {
        return this == RADIANS ? inOurUnits : Math.toRadians(inOurUnits);
    }

    public double normalize(double angle) {
        if (this == DEGREES) {
            while (angle >= 180) angle -= 360;
            while (angle < -180) angle += 360;
        } else {
            while (angle >= Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
        }
        return angle;
    }

    public static double normalizeDegrees(double degrees) {
        return DEGREES.normalize(degrees);
    }

    public static double normalizeRadians(double radians) {
        return RADIANS.normalize(radians);
    }
}
