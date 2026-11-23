package org.firstinspires.ftc.robotcore.external.hardware.camera.controls;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface GainControl extends CameraControl {
    int getMinGain();

    int getMaxGain();

    int getGain();

    boolean setGain(int gain);
}
