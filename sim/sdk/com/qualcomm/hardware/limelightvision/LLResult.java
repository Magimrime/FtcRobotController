package com.qualcomm.hardware.limelightvision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** SIMULATOR stand-in for the FTC SDK class: one picture's results. */
public class LLResult {
    private final double timestampMs;
    private final int pipelineIndex;
    private final String pipelineType;
    private final List<LLResultTypes.FiducialResult> fiducials;
    private final double captureLatencyMs, targetingLatencyMs;
    private final long controlHubTimeStampNanos;

    /** Simulator: built by ftcsim.CameraModel. */
    public LLResult(double timestampMs, int pipelineIndex, String pipelineType, List<LLResultTypes.FiducialResult> fiducials,
                    double captureLatencyMs, double targetingLatencyMs, long controlHubTimeStampNanos) {
        this.timestampMs = timestampMs;
        this.pipelineIndex = pipelineIndex;
        this.pipelineType = pipelineType;
        this.fiducials = Collections.unmodifiableList(new ArrayList<>(fiducials));
        this.captureLatencyMs = captureLatencyMs;
        this.targetingLatencyMs = targetingLatencyMs;
        this.controlHubTimeStampNanos = controlHubTimeStampNanos;
    }

    public double getTimestamp() {
        return timestampMs;
    }

    public int getPipelineIndex() {
        return pipelineIndex;
    }

    public String getPipelineType() {
        return pipelineType;
    }

    /** The real camera reports v = 1 when it has at least one target. */
    public boolean isValid() {
        return !fiducials.isEmpty();
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        return fiducials;
    }

    public List<LLResultTypes.ColorResult> getColorResults() {
        return new ArrayList<>();
    }

    public double getCaptureLatency() {
        return captureLatencyMs;
    }

    public double getTargetingLatency() {
        return targetingLatencyMs;
    }

    public double getParseLatency() {
        return 0.1;
    }

    public long getControlHubTimeStamp() {
        return controlHubTimeStampNanos / 1000000;
    }

    public long getControlHubTimeStampNanos() {
        return controlHubTimeStampNanos;
    }

    /** Milliseconds since the result arrived on the Control Hub. */
    public long getStaleness() {
        return (ftcsim.SimContext.get().nanos() - controlHubTimeStampNanos) / 1000000;
    }

    public double getTx() {
        return fiducials.isEmpty() ? 0 : fiducials.get(0).getTargetXDegrees();
    }

    public double getTy() {
        return fiducials.isEmpty() ? 0 : fiducials.get(0).getTargetYDegrees();
    }

    public double getTxNC() {
        return getTx();
    }

    public double getTyNC() {
        return getTy();
    }

    public double getTa() {
        return fiducials.isEmpty() ? 0 : fiducials.get(0).getTargetArea();
    }

    public Pose3D getBotpose() {
        return LLResultTypes.zeroPose();
    }

    public Pose3D getBotpose_MT2() {
        return LLResultTypes.zeroPose();
    }

    public int getBotposeTagCount() {
        return 0;
    }

    public double[] getPythonOutput() {
        return new double[0];
    }

    @Override
    public String toString() {
        return "LLResult{ts=" + timestampMs + ", pipeline=" + pipelineIndex + ", tags=" + fiducials.size() + "}";
    }
}
