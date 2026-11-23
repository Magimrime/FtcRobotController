package com.qualcomm.hardware.rev;

import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

/** SIMULATOR stand-in for the FTC SDK class (the simulated IMU is always mounted as declared). */
public class RevHubOrientationOnRobot implements ImuOrientationOnRobot {
    public enum LogoFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }

    public enum UsbFacingDirection { UP, DOWN, FORWARD, BACKWARD, LEFT, RIGHT }

    public final LogoFacingDirection logo;
    public final UsbFacingDirection usb;

    public RevHubOrientationOnRobot(LogoFacingDirection logo, UsbFacingDirection usb) {
        this.logo = logo;
        this.usb = usb;
    }
}
