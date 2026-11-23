package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.ArrayList;

/**
 * SIMULATOR stand-in for the FTC SDK AprilTagProcessor. Builder.build() returns the simulator's tag
 * finder (ftcsim.SimAprilTagProcessor), fed by the simulated webcam. As in the SDK, getFreshDetections()
 * returns null when no new picture has been processed since the last call.
 */
public abstract class AprilTagProcessor implements VisionProcessor {
    public static final int THREADS_DEFAULT = 3;

    public enum PoseSolver { APRILTAG_BUILTIN, OPENCV_ITERATIVE, OPENCV_SOLVEPNP_EPNP, OPENCV_IPPE, OPENCV_IPPE_SQUARE, OPENCV_SQPNP }

    public enum TagFamily { TAG_36h11, TAG_25h9, TAG_16h5, TAG_standard41h12 }

    public abstract void setDecimation(float decimation);

    public abstract void setPoseSolver(PoseSolver solver);

    public abstract int getPerTagAvgPoseSolveTime();

    public abstract ArrayList<AprilTagDetection> getDetections();

    public abstract ArrayList<AprilTagDetection> getFreshDetections();

    public static class Builder {
        private AprilTagLibrary library;
        private DistanceUnit distanceUnit = DistanceUnit.INCH;
        private AngleUnit angleUnit = AngleUnit.DEGREES;

        public Builder setTagLibrary(AprilTagLibrary library) {
            this.library = library;
            return this;
        }

        public Builder setTagFamily(TagFamily family) {
            return this;
        }

        public Builder setOutputUnits(DistanceUnit distanceUnit, AngleUnit angleUnit) {
            this.distanceUnit = distanceUnit;
            this.angleUnit = angleUnit;
            return this;
        }

        public Builder setLensIntrinsics(double fx, double fy, double cx, double cy) {
            return this;
        }

        public Builder setDrawAxes(boolean draw) {
            return this;
        }

        public Builder setDrawCubeProjection(boolean draw) {
            return this;
        }

        public Builder setDrawTagOutline(boolean draw) {
            return this;
        }

        public Builder setDrawTagID(boolean draw) {
            return this;
        }

        public Builder setNumThreads(int threads) {
            return this;
        }

        public AprilTagProcessor build() {
            return new ftcsim.SimAprilTagProcessor(library, distanceUnit, angleUnit);
        }
    }
}
