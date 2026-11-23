package org.firstinspires.ftc.robotcore.external.navigation;

/** SIMULATOR copy of the FTC SDK enum. */
public enum CurrentUnit {
    AMPS, MILLIAMPS;

    public double convert(double value, CurrentUnit from) {
        if (from == this) return value;
        return this == AMPS ? value / 1000 : value * 1000;
    }

    public double toAmps(double value) {
        return this == AMPS ? value : value / 1000;
    }

    public double toMilliAmps(double value) {
        return this == MILLIAMPS ? value : value * 1000;
    }
}
