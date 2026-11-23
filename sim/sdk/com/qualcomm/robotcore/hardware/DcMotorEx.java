package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface DcMotorEx extends DcMotor {
    void setMotorEnable();

    void setMotorDisable();

    boolean isMotorEnabled();

    /** ticks per second */
    void setVelocity(double angularRate);

    void setVelocity(double angularRate, AngleUnit unit);

    /** ticks per second */
    double getVelocity();

    double getVelocity(AngleUnit unit);

    void setVelocityPIDFCoefficients(double p, double i, double d, double f);

    void setPositionPIDFCoefficients(double p);

    void setTargetPositionTolerance(int tolerance);

    int getTargetPositionTolerance();

    /** Motor current (a hub command of its own, not in the bulk read). */
    double getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit unit);

    double getCurrentAlert(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit unit);

    void setCurrentAlert(double current, org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit unit);

    boolean isOverCurrent();
}
