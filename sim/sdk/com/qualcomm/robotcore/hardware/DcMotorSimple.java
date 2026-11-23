package com.qualcomm.robotcore.hardware;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface DcMotorSimple extends HardwareDevice {
    enum Direction {
        FORWARD, REVERSE;

        public Direction inverted() {
            return this == FORWARD ? REVERSE : FORWARD;
        }
    }

    void setDirection(Direction direction);

    Direction getDirection();

    void setPower(double power);

    double getPower();
}
