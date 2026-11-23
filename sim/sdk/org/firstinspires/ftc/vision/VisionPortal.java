package org.firstinspires.ftc.vision;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;

import java.util.ArrayList;
import java.util.List;

/**
 * SIMULATOR stand-in for the FTC SDK VisionPortal. Builder.build() returns the simulator's webcam
 * (ftcsim.SimVisionPortal): it opens like a real one (OPENING_CAMERA_DEVICE ... STREAMING after
 * about 1.5 s) and feeds its AprilTag processor pictures of the simulated field.
 */
public abstract class VisionPortal {
    public static final int DEFAULT_VIEW_CONTAINER_ID = 0;

    public enum CameraState {
        OPENING_CAMERA_DEVICE, CAMERA_DEVICE_READY, STARTING_STREAM, STREAMING, STOPPING_STREAM,
        CLOSING_CAMERA_DEVICE, CAMERA_DEVICE_CLOSED, ERROR
    }

    public enum StreamFormat { YUY2, MJPEG }

    public abstract void setProcessorEnabled(VisionProcessor processor, boolean enabled);

    public abstract boolean getProcessorEnabled(VisionProcessor processor);

    public abstract CameraState getCameraState();

    public abstract void saveNextFrameRaw(String filename);

    public abstract void stopStreaming();

    public abstract void resumeStreaming();

    public abstract void stopLiveView();

    public abstract void resumeLiveView();

    public abstract float getFps();

    public abstract <T extends CameraControl> T getCameraControl(Class<T> controlType);

    public abstract void setActiveCamera(WebcamName webcamName);

    public abstract WebcamName getActiveCamera();

    public abstract void close();

    public static class Builder {
        private CameraName camera;
        private Size size = new Size(640, 480);
        private final List<VisionProcessor> processors = new ArrayList<>();

        public Builder setCamera(CameraName camera) {
            this.camera = camera;
            return this;
        }

        public Builder setCameraResolution(Size size) {
            this.size = size;
            return this;
        }

        public Builder setStreamFormat(StreamFormat format) {
            return this;
        }

        public Builder enableLiveView(boolean enable) {
            return this;
        }

        public Builder setAutoStopLiveView(boolean autoStop) {
            return this;
        }

        public Builder setAutoStartStreamOnBuild(boolean autoStart) {
            return this;
        }

        public Builder setShowStatsOverlay(boolean show) {
            return this;
        }

        public Builder addProcessor(VisionProcessor processor) {
            processors.add(processor);
            return this;
        }

        public Builder addProcessors(VisionProcessor... list) {
            for (VisionProcessor p : list) processors.add(p);
            return this;
        }

        public VisionPortal build() {
            if (camera == null) {
                throw new RuntimeException("A camera must be specified");
            }
            return new ftcsim.SimVisionPortal(camera, size, processors);
        }
    }
}
