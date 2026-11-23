package com.qualcomm.robotcore.hardware;

/** SIMULATOR stand-in for the FTC SDK interface. */
public interface HardwareDevice {
    enum Manufacturer { Unknown, Other, Lego, HiTechnic, ModernRobotics, Adafruit, Matrix, Lynx, AMS, STMicroelectronics, Broadcom, DFRobot, SparkFun, Limelight }

    default Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    default String getDeviceName() {
        return "simulated device";
    }

    default String getConnectionInfo() {
        return "simulator";
    }

    default int getVersion() {
        return 1;
    }

    default void resetDeviceConfigurationForOpMode() {
    }

    default void close() {
    }
}
