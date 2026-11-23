package com.qualcomm.hardware.limelightvision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

/** SIMULATOR stand-in for the FTC SDK class (AprilTag results only; color/detector lists stay empty). */
public class LLResultTypes {

    static Pose3D zeroPose() {
        return new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    }

    public static class FiducialResult {
        private final int id;
        private final Pose3D targetPoseCameraSpace;
        private final double txDeg, tyDeg, area, txPx, tyPx;

        /** Simulator: one detected tag. Pose in METERS, Limelight camera space (x right, y down, z out of the lens). */
        public FiducialResult(int id, Pose3D targetPoseCameraSpace, double txDeg, double tyDeg, double area, double txPx, double tyPx) {
            this.id = id;
            this.targetPoseCameraSpace = targetPoseCameraSpace;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.area = area;
            this.txPx = txPx;
            this.tyPx = tyPx;
        }

        public int getFiducialId() {
            return id;
        }

        public String getFamily() {
            return "36H11C";
        }

        public List<List<Double>> getTargetCorners() {
            return new ArrayList<>();
        }

        public double getSkew() {
            return 0;
        }

        public Pose3D getTargetPoseCameraSpace() {
            return targetPoseCameraSpace;
        }

        public Pose3D getCameraPoseTargetSpace() {
            return zeroPose();
        }

        public Pose3D getRobotPoseFieldSpace() {
            return zeroPose();
        }

        public Pose3D getRobotPoseTargetSpace() {
            return zeroPose();
        }

        public Pose3D getTargetPoseRobotSpace() {
            return zeroPose();
        }

        public double getTargetArea() {
            return area;
        }

        public double getTargetXPixels() {
            return txPx;
        }

        public double getTargetYPixels() {
            return tyPx;
        }

        public double getTargetXDegrees() {
            return txDeg;
        }

        public double getTargetYDegrees() {
            return tyDeg;
        }

        public double getTargetXDegreesNoCrosshair() {
            return txDeg;
        }

        public double getTargetYDegreesNoCrosshair() {
            return tyDeg;
        }
    }

    /** Present so code that asks for color results compiles; the simulator never fills it. */
    public static class ColorResult {
        public double getTargetXDegrees() {
            return 0;
        }

        public double getTargetYDegrees() {
            return 0;
        }

        public double getTargetArea() {
            return 0;
        }

        public List<List<Double>> getTargetCorners() {
            return new ArrayList<>();
        }
    }
}
