package org.firstinspires.ftc.robotcore.external.hardware.camera;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface CameraName {
    default boolean isWebcam() {
        return true;
    }
}
