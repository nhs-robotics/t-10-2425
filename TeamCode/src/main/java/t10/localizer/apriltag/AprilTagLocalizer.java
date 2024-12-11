package t10.localizer.apriltag;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import t10.geometry.Point;
import t10.utils.MathUtils;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class AprilTagLocalizer {
    public final AprilTagProcessor aprilTagProcessor;

    /**
     * Only April Tags closer than this distance to the robot will be considered.
     */
    private final double distanceRejectThresholdIn;

    public AprilTagLocalizer(double fx, double fy, double px, double py) {
        this(AprilTagProcessor.TagFamily.TAG_36h11, true, fx, fy, px, py, 18);
    }

    public AprilTagLocalizer(AprilTagProcessor.TagFamily tagFamily, boolean draw, double focalLengthX, double focalLengthY, double principalPointX, double principalPointY, double distanceRejectThresholdIn) {
        this.distanceRejectThresholdIn = distanceRejectThresholdIn;
        this.aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(draw)
                .setDrawTagID(draw)
                .setDrawTagOutline(draw)
                .setDrawCubeProjection(draw)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)  // inches and degrees match the april tag library metadata
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setTagFamily(tagFamily)
                .setCameraPose(
                        new Position(DistanceUnit.INCH, 0, 0 , 0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0)
                )
                .setLensIntrinsics(focalLengthX, focalLengthY, principalPointX, principalPointY)
                .build();

    }

    public Point getFieldLocation() {
        ArrayList<AprilTagDetection> detections = this.aprilTagProcessor.getDetections();

        if (detections.isEmpty()) {
            return null;
        }

        List<Double> x = new LinkedList<>();
        List<Double> y = new LinkedList<>();
        List<Double> weights = new LinkedList<>();

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose == null || detection.ftcPose.range > this.distanceRejectThresholdIn) {
                continue;
            }

            Position position = detection.robotPose.getPosition();

            // Transform AprilTag coordinate to our local coordinate system.
            double localCoordinateSysX = -position.y;  // +X in our coordinate system = -y in AprilTag coordinate system
            double localCoordinateSysY = -position.x;   // +Y in our coordinate system = +x in AprilTag coordinate system

            x.add(localCoordinateSysX);
            y.add(localCoordinateSysY);
            weights.add((double) detection.decisionMargin);
        }

        if (weights.isEmpty()) {
            return null;
        }

        double fcPositionX = MathUtils.weightedAverage(x, weights);
        double fcPositionY = MathUtils.weightedAverage(y, weights);

        return new Point(fcPositionX, fcPositionY);
    }
}
