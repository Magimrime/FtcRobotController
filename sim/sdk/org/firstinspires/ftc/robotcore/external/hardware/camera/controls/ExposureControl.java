package org.firstinspires.ftc.robotcore.external.hardware.camera.controls;

import java.util.concurrent.TimeUnit;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface ExposureControl extends CameraControl {
    enum Mode { Unknown, Auto, Manual, ShutterPriority, AperturePriority, ContinuousAuto }

    Mode getMode();

    boolean setMode(Mode mode);

    boolean isModeSupported(Mode mode);

    long getMinExposure(TimeUnit resultUnit);

    long getMaxExposure(TimeUnit resultUnit);

    long getExposure(TimeUnit resultUnit);

    boolean setExposure(long duration, TimeUnit durationUnit);

    boolean isExposureSupported();

    boolean getAePriority();

    boolean setAePriority(boolean priority);
}
