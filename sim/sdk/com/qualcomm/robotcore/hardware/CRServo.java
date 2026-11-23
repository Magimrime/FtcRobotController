package com.qualcomm.robotcore.hardware;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface CRServo extends DcMotorSimple {
    int getPortNumber();
}
