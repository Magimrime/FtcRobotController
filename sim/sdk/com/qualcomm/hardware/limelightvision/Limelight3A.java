package com.qualcomm.hardware.limelightvision;

import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.json.JSONObject;

import ftcsim.CameraModel;
import ftcsim.SimContext;

/**
 * SIMULATOR stand-in for the FTC SDK Limelight3A. Same public API, plus the two private methods the
 * team reaches by reflection (getPipelineAtIndex, updatePipeline). The pictures come from
 * ftcsim.CameraModel: real AprilTag geometry of both HIVEs seen from the robot's true pose, with
 * the camera's frame rate, latency, field of view, detection limits, noise and dropouts.
 * Calls that are HTTP requests on the real camera (status, pipeline switch, upload) block for a
 * few tens of milliseconds of simulated time, like the real ones.
 */
public class Limelight3A implements HardwareDevice {
    private final CameraModel cam;
    private final String name;
    private boolean running = false;
    private int pollRateHz = 100;

    public Limelight3A(CameraModel cam, String name) {
        this.cam = cam;
        this.name = name;
    }

    private void http() {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costHttp);
    }

    public synchronized void start() {
        running = true;
        cam.setRunning(true);
    }

    public synchronized void pause() {
        running = false;
        cam.setRunning(false);
    }

    public synchronized void stop() {
        running = false;
        cam.setRunning(false);
    }

    public boolean isRunning() {
        return running;
    }

    public synchronized void setPollRateHz(int rateHz) {
        pollRateHz = Math.max(1, Math.min(250, rateHz));
        cam.setPollRateHz(pollRateHz);
    }

    public long getTimeSinceLastUpdate() {
        return cam.msSinceLastUpdate();
    }

    public boolean isConnected() {
        return cam.isConnected();
    }

    public LLResult getLatestResult() {
        SimContext c = SimContext.get();
        c.tick(c.scenario.costCameraRead);
        return running ? cam.latestResult() : cam.lastResultEver();
    }

    public LLStatus getStatus() {
        http();
        return new LLStatus(cam.activePipelineType(), cam.activePipeline(), cam.fps());
    }

    public boolean reloadPipeline() {
        http();
        return cam.reloadPipeline();
    }

    public boolean pipelineSwitch(int index) {
        http();
        return cam.switchPipeline(index);
    }

    private JSONObject getPipelineAtIndex(int index) {
        http();
        return cam.pipelineJson(index);
    }

    private boolean updatePipeline(JSONObject profileJson, boolean flush) {
        http();
        return cam.updatePipeline(profileJson, flush);
    }

    public boolean uploadPipeline(String jsonString, Integer index) {
        http();
        return cam.uploadPipeline(jsonString, index);
    }

    public boolean captureSnapshot(String snapname) {
        http();
        return true;
    }

    public boolean updateRobotOrientation(double yaw) {
        return true;
    }

    public void shutdown() {
        stop();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Limelight;
    }

    @Override
    public String getDeviceName() {
        return "Limelight3A (simulated) " + name;
    }
}
