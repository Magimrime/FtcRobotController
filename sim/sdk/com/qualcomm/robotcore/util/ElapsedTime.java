package com.qualcomm.robotcore.util;

import java.util.concurrent.TimeUnit;

/**
 * SIMULATOR stand-in for the FTC SDK ElapsedTime: the same API, but it reads the simulated clock,
 * so a 30 s AUTO runs in well under a second and every run is repeatable.
 */
public class ElapsedTime {
    public enum Resolution { SECONDS, MILLISECONDS }

    public static final long SECOND_IN_NANO = 1000000000;
    public static final long MILLIS_IN_NANO = 1000000;

    protected volatile long nsStartTime;
    protected final double resolution;

    public ElapsedTime() {
        reset();
        resolution = SECOND_IN_NANO;
    }

    public ElapsedTime(long startTime) {
        nsStartTime = startTime;
        resolution = SECOND_IN_NANO;
    }

    public ElapsedTime(Resolution resolution) {
        reset();
        this.resolution = resolution == Resolution.MILLISECONDS ? MILLIS_IN_NANO : SECOND_IN_NANO;
    }

    protected long nsNow() {
        ftcsim.SimContext c = ftcsim.SimContext.getOrNull();
        return c == null ? System.nanoTime() : c.nanos();
    }

    public long now(TimeUnit unit) {
        return unit.convert(nsNow(), TimeUnit.NANOSECONDS);
    }

    public void reset() {
        nsStartTime = nsNow();
    }

    public double startTime() {
        return nsStartTime / resolution;
    }

    public long startTimeNanoseconds() {
        return nsStartTime;
    }

    public double time() {
        return (nsNow() - nsStartTime) / resolution;
    }

    public long time(TimeUnit unit) {
        return unit.convert(nanoseconds(), TimeUnit.NANOSECONDS);
    }

    public double seconds() {
        return nanoseconds() / (double) SECOND_IN_NANO;
    }

    public double milliseconds() {
        return seconds() * 1000;
    }

    public long nanoseconds() {
        return nsNow() - nsStartTime;
    }

    public Resolution getResolution() {
        return resolution == MILLIS_IN_NANO ? Resolution.MILLISECONDS : Resolution.SECONDS;
    }

    public void log(String label) {
    }

    @Override
    public String toString() {
        return String.format("%.4f seconds", seconds());
    }
}
