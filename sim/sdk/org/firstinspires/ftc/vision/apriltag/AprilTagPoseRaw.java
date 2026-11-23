package org.firstinspires.ftc.vision.apriltag;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

/** SIMULATOR copy of the FTC SDK class: tag position in the camera frame (x right, y down, z out of the lens), output units. */
public class AprilTagPoseRaw {
    public final double x, y, z;
    public final MatrixF R;

    public AprilTagPoseRaw(double x, double y, double z, MatrixF R) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.R = R;
    }
}
