package ftcsim;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseRaw;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * The SDK's AprilTag processor, fed by SimVisionPortal. Reports only tags in its library, as single
 * detections, in the output units the code asked for; getFreshDetections() is null until a new
 * picture has been processed, like the SDK's.
 */
public class SimAprilTagProcessor extends AprilTagProcessor {
    final AprilTagLibrary library;
    final DistanceUnit distanceUnit;
    final AngleUnit angleUnit;
    float decimation = 3;  // the SDK's default
    private ArrayList<AprilTagDetection> latest = new ArrayList<>();
    private boolean fresh = false;

    public SimAprilTagProcessor(AprilTagLibrary library, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        this.library = library;
        this.distanceUnit = distanceUnit;
        this.angleUnit = angleUnit;
    }

    /** A processed picture: these tags were found in it. */
    void deliver(List<CameraModel.TagSeen> seen, long frameNanos) {
        ArrayList<AprilTagDetection> list = new ArrayList<>();
        for (CameraModel.TagSeen o : seen) {
            AprilTagMetadata meta = library == null ? null : library.lookupTag(o.id);
            if (meta == null) continue;  // the SDK only reports tags in its library
            double x = distanceUnit.fromInches(o.x), y = distanceUnit.fromInches(o.y), z = distanceUnit.fromInches(o.z);
            AprilTagPoseRaw raw = new AprilTagPoseRaw(x, y, z, new MatrixF(3, 3));
            // FTC pose: x right, y forward (out of the lens), z up
            double fx = x, fy = z, fz = -y;
            double range = Math.sqrt(fx * fx + fy * fy + fz * fz);
            AprilTagPoseFtc ftc = new AprilTagPoseFtc(fx, fy, fz, 0, 0, 0, range,
                    angleUnit.fromDegrees(Math.toDegrees(-Math.atan2(fx, fy))), angleUnit.fromDegrees(Math.toDegrees(Math.atan2(fz, fy))));
            double half = o.sizePx / 2;
            Point[] corners = {new Point(o.u - half, o.v + half), new Point(o.u + half, o.v + half),
                    new Point(o.u + half, o.v - half), new Point(o.u - half, o.v - half)};
            list.add(new AprilTagSingleDetection(o.id, 0, (float) Math.min(90, 3 * o.shortPx), new Point(o.u, o.v), corners, meta,
                    ftc, raw, null, frameNanos, distanceUnit));
        }
        latest = list;
        fresh = true;
    }

    int latestCount() {
        return latest.size();
    }

    @Override
    public void setDecimation(float decimation) {
        this.decimation = Math.max(1, decimation);
    }

    @Override
    public void setPoseSolver(PoseSolver solver) {
    }

    @Override
    public int getPerTagAvgPoseSolveTime() {
        return 2;
    }

    @Override
    public ArrayList<AprilTagDetection> getDetections() {
        return new ArrayList<>(latest);
    }

    @Override
    public ArrayList<AprilTagDetection> getFreshDetections() {
        if (!fresh) return null;
        fresh = false;
        return new ArrayList<>(latest);
    }
}
