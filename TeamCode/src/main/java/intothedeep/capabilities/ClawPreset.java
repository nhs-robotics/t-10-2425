package intothedeep.capabilities;

public enum ClawPreset {
	STRAIGHT_FORWARD(0, 0),
	PLACE_SPECIMEN(1, 0),
	COLLECT_SPECIMEN_FROM_WALL(-1, 0);

	public final double servoRotatePosition;
	public final double servoTwistPosition;

	ClawPreset(double servoRotatePosition, double servoTwistPosition) {
		this.servoRotatePosition = servoRotatePosition;
		this.servoTwistPosition = servoTwistPosition;
	}
}
