package com.qualcomm.hardware.limelightvision;

/** SIMULATOR stand-in for the FTC SDK class. */
public class LLStatus {
    private final String pipelineType;
    private final int pipelineIndex;
    private final double fps;

    public LLStatus() {
        this("", 0, 0);
    }

    public LLStatus(String pipelineType, int pipelineIndex, double fps) {
        this.pipelineType = pipelineType;
        this.pipelineIndex = pipelineIndex;
        this.fps = fps;
    }

    public String getPipelineType() {
        return pipelineType;
    }

    public int getPipelineIndex() {
        return pipelineIndex;
    }

    public double getFps() {
        return fps;
    }

    public double getTemp() {
        return 45;
    }

    public double getCpu() {
        return 30;
    }

    public double getRam() {
        return 40;
    }

    public String getName() {
        return "limelight";
    }

    public int getHwType() {
        return 0;
    }

    public int getPipeImgCount() {
        return 0;
    }

    public int getSnapshotMode() {
        return 0;
    }

    public int getCid() {
        return 0;
    }

    public double getFinalYaw() {
        return 0;
    }

    @Override
    public String toString() {
        return "LLStatus{pipelineType=" + pipelineType + ", index=" + pipelineIndex + ", fps=" + fps + "}";
    }
}
