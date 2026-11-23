package com.qualcomm.robotcore.hardware;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/** SIMULATOR stand-in for the FTC SDK interface (the parts the team's OpModes use). */
public interface IMU extends HardwareDevice {
    class Parameters {
        public ImuOrientationOnRobot imuOrientationOnRobot;

        public Parameters(ImuOrientationOnRobot imuOrientationOnRobot) {
            this.imuOrientationOnRobot = imuOrientationOnRobot;
        }
    }

    boolean initialize(Parameters parameters);

    void resetYaw();

    YawPitchRollAngles getRobotYawPitchRollAngles();
}
