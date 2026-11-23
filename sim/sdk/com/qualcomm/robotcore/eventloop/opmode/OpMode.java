package com.qualcomm.robotcore.eventloop.opmode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** SIMULATOR stand-in for the FTC SDK class of the same name (only what the team's OpModes use). */
public abstract class OpMode {
    public Gamepad gamepad1 = new Gamepad();
    public Gamepad gamepad2 = new Gamepad();
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public double time = 0;
    private double startTime = 0;

    public double getRuntime() {
        return ftcsim.SimContext.get().now() - startTime;
    }

    public void resetRuntime() {
        startTime = ftcsim.SimContext.get().now();
    }

    public void requestOpModeStop() {
        ftcsim.SimContext.get().requestStop("OpMode called requestOpModeStop()");
    }
}
