package org.firstinspires.ftc.robotcore.external.hardware.camera;

import com.qualcomm.robotcore.hardware.HardwareDevice;

/** SIMULATOR stand-in for the FTC SDK interface: a USB webcam in the robot configuration. */
public interface WebcamName extends CameraName, HardwareDevice {
    default boolean isAttached() {
        return true;
    }
}
