package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.opencv.core.Point;

/** SIMULATOR copy of the FTC SDK 12 class. */
public class AprilTagSingleDetection extends AprilTagDetection {
    public final int id;
    public final int hamming;
    public final float decisionMargin;
    public final Point[] corners;
    public final Point center;
    public final AprilTagMetadata metadata;

    public AprilTagSingleDetection(int id, int hamming, float decisionMargin, Point center, Point[] corners, AprilTagMetadata metadata,
                                   AprilTagPoseFtc ftcPose, AprilTagPoseRaw rawPose, Pose3D robotPose, long frameAcquisitionNanoTime,
                                   DistanceUnit distanceUnit) {
        super(ftcPose, rawPose, robotPose, frameAcquisitionNanoTime, distanceUnit);
        this.id = id;
        this.hamming = hamming;
        this.decisionMargin = decisionMargin;
        this.center = center;
        this.corners = corners;
        this.metadata = metadata;
    }
}
