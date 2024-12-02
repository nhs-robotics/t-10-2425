package t10.novel.odometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import t10.novel.AbstractEncoder;
import t10.novel.NovelEncoder;
import t10.reconstructor.Pose;
import t10.utils.MovementVector;

/**
 * Odometry localization interface.
 */
public class NovelOdometry {
    private final OdometryCoefficientSet coefficients;
    private final AbstractEncoder rightEncoder;
    private final AbstractEncoder leftEncoder;
    private final AbstractEncoder perpendicularEncoder;
    private final double lateralWheelDistance;
    private final double perpendicularWheelOffset;
    private double leftWheelPos;
    private double rightWheelPos;
    private double perpendicularWheelPos;
    private Pose relativePose;

    /**
     * Creates a NovelOdometry instance.
     * See diagram: <a href="http://web.archive.org/web/20230529000105if_/https://gm0.org/en/latest/_images/offsets-and-trackwidth.png">here (archived)</a> or <a href="https://gm0.org/en/latest/_images/offsets-and-trackwidth.png">here</a>.
     *
     * @param coefficients The coefficients to use for the odometers. Chances are this is {@link OdometryCoefficientSet#DEFAULT}.
     * @param rightEncoder The right side encoder.
     * @param leftEncoder The left side encoder.
     * @param perpendicularEncoder The perpendicular encoder.
     * @param lateralWheelDistance The distance between the lateral wheels.
     * @param perpendicularWheelOffset The offset of the perpendicular wheel from the center of the robot chassis.
     */
    public NovelOdometry(
            OdometryCoefficientSet coefficients,
            AbstractEncoder rightEncoder,
            AbstractEncoder leftEncoder,
            AbstractEncoder perpendicularEncoder,
            double lateralWheelDistance,
            double perpendicularWheelOffset
    ) {
        this.coefficients = coefficients;
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.perpendicularEncoder = perpendicularEncoder;
        this.lateralWheelDistance = lateralWheelDistance;
        this.perpendicularWheelOffset = perpendicularWheelOffset;

        this.setRelativePose(new Pose(0, 0, 0, AngleUnit.RADIANS));
    }

    /**
     * Updates the underlying state of this odometry localizer.
     * Calling this method more regularly increases accuracy of the calculations, but obviously it is impossible to call this method infinitely fast, so calculations will contain some error.
     * <p>
     * Adapted from <a href="https://gm0.org/en/latest/docs/software/concepts/odometry.html">gm0</a>.
     */
    public void update() {
        // Get new wheel positions
        double newLeftWheelPos = this.leftEncoder.getCurrentInches();
        double newRightWheelPos = this.rightEncoder.getCurrentInches();
        double newPerpendicularWheelPos = this.perpendicularEncoder.getCurrentInches();

        // Get changes in odometry wheel positions since last update - results from the robot's perspective
        double deltaLeftWheelPos = this.coefficients.leftCoefficient * (newLeftWheelPos - this.leftWheelPos);
        double deltaRightWheelPos = this.coefficients.rightCoefficient * (newRightWheelPos - this.rightWheelPos);
        double deltaPerpendicularWheelPos = this.coefficients.perpendicularCoefficient * (newPerpendicularWheelPos - this.perpendicularWheelPos);

        // Convert changes in robot-perspective wheel positions into changes in x/y/angle from the robot's perspective
        double phi = (deltaLeftWheelPos - deltaRightWheelPos) / this.lateralWheelDistance;
        double forwardRelative = (deltaLeftWheelPos + deltaRightWheelPos) / 2d;
        double rightwardRelative = deltaPerpendicularWheelPos - this.perpendicularWheelOffset * phi;

        // Computes the robot's new  heading for purposes of trig
        double heading = this.relativePose.getHeading(AngleUnit.RADIANS) - phi;

        //converts x and y positions from robot-relative to field-relative
        double deltaX = forwardRelative * -Math.sin(heading) + rightwardRelative * Math.cos(heading);
        double deltaY = forwardRelative * Math.cos(heading) +  rightwardRelative * Math.sin(heading);

        // Updates the Pose (position + heading)
        this.relativePose = this.relativePose.add(new Pose(-deltaY, deltaX, -phi, AngleUnit.RADIANS));

        // Update encoder wheel position
        this.leftWheelPos = newLeftWheelPos;
        this.rightWheelPos = newRightWheelPos;
        this.perpendicularWheelPos = newPerpendicularWheelPos;
    }

    /**
     * @return The current relative pose of the robot based on the odometers.
     */
    public Pose getRelativePose() {
        return this.relativePose;
    }

    /**
     * Sets the relative pose to a new pose. Helps to combat error as a result of {@link NovelOdometry#update()} not being called infinitely fast.
     *
     * @param pose New relative pose to base odometry calculations off of.
     */
    public void setRelativePose(Pose pose) {
        this.relativePose = pose;
        this.leftWheelPos = this.leftEncoder.getCurrentInches();
        this.rightWheelPos = this.rightEncoder.getCurrentInches();
        this.perpendicularWheelPos = this.perpendicularEncoder.getCurrentInches();
    }

    public MovementVector getRelativeVelocity(MovementVector absoluteVelocity)
    {
        double theta = relativePose.getHeading(AngleUnit.RADIANS);
        double forwardRelative = -absoluteVelocity.getVertical() * Math.cos(theta) - absoluteVelocity.getHorizontal() * Math.sin(theta);
        double rightwardRelative = -absoluteVelocity.getVertical() * Math.sin(theta) + absoluteVelocity.getHorizontal() * Math.cos(theta);
        return new MovementVector(forwardRelative,rightwardRelative, 0);
    }

    //IMPORTANT - this MUST be iterated, otherwise it'll keep going in the earlier direction while rotating - and not work
    public MovementVector getRelativeVelocity(double lateral, double horizontal)
    {
        return getRelativeVelocity(new MovementVector(lateral,horizontal, 0));
    }
}
