package intothedeep.teleop.azazel;

import t10.bootstrap.AbstractRobotConfiguration;
import t10.bootstrap.Hardware;
import t10.localizer.odometry.OdometryLocalizer;
import t10.motion.hardware.Motor;
import t10.motion.mecanum.MecanumDriver;
import t10.vision.Webcam;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import intothedeep.Constants;

public class AzazelRobotConfiguration extends AbstractRobotConfiguration {
    @Hardware(name = "Webcam")
    public Webcam webcam;

    /***************\
     |* CONTROL HUB *|
     \***************/

    // Wheels
    // NOTE: LinearSlideLeft has the perpendicular  odometer encoder
    @Hardware(name = "LinearSlideLeft")
    public Motor linearSlideLeft;

    @Hardware(name = "LinearSlideRight")
    public Motor linearSlideRight;

    // NOTE: LinearSlideLeft has the right odometer encoder
    @Hardware(name = "SpinningIntake")
    public Motor spinningIntake;

    // NOTE: LinearSlideLeft has the left odometer encoder
    @Hardware(name = "Roller")
    public Motor roller;

    // Servos
    @Hardware(name = "AirplaneLauncher")
    public Servo airplaneLauncher;

    /*****************\
     |* EXPANSION HUB *|
     \*****************/

    // Wheels
    @Hardware(name = "FL", diameterIn = 3.7795275590551185, ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT, zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE)
    public Motor fl;

    @Hardware(name = "FR", diameterIn = 3.7795275590551185, ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT, zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE)
    public Motor fr;

    @Hardware(name = "BL", diameterIn = 3.7795275590551185, ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT, zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE)
    public Motor bl;

    @Hardware(name = "BR", diameterIn = 3.7795275590551185, ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT, zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE)
    public Motor br;

    // Servos
    @Hardware(name = "ContainerPixelHolder")
    public Servo containerPixelHolder;

    @Hardware(name = "ContainerRotationLeft")
    // NOTE: The left servo is able to rotate the container downwards from halfway, but not upwards from halfway.
    public Servo containerRotationLeft;

    @Hardware(name = "ContainerRotationRight")
    // NOTE: The right servo is able to rotate the container upwards from halfway, but not downwards from halfway.
    public Servo containerRotationRight;

    @Hardware(name = "imu")
    public IMU imu;

    public AzazelRobotConfiguration(HardwareMap hardwareMap) {
        super(hardwareMap);
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ))
        );
    }

    @Override
    public MecanumDriver createMecanumDriver() {
        return new MecanumDriver(fl, fr, bl, br, Constants.Coefficients.KEVIN_COEFFICIENTS);
    }

    @Override
    public OdometryLocalizer createOdometry() {
        return null;
        //return new OdometryLocalizer(OdometryCoefficientSet.DEFAULT, spinningIntake.encoder, roller.encoder, linearSlideLeft.encoder, Constants.Odometry.ODOMETRY_LATERAL_WHEEL_DISTANCE, Constants.Odometry.ODOMETRY_PERPENDICULAR_WHEEL_OFFSET);
    }
}
