package intothedeep;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import t10.bootstrap.AbstractRobotConfiguration;
import t10.bootstrap.Hardware;
import t10.motion.NovelEncoder;
import t10.novel.OdometryEncoder;
import t10.localizer.odometry.OdometryLocalizer;
import t10.motion.NovelEncoder;
import t10.motion.NovelMotor;
import t10.motion.mecanum.MecanumDriver;
import t10.localizer.odometry.OdometryCoefficientSet;

public class IntoTheDeepRobotConfiguration extends AbstractRobotConfiguration {

    // Wheels
    @Hardware(
            name = "FL",
            diameterIn = Constants.Robot.ACTUAL_DIAMETER_IN,
            ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT,
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    )
    public NovelMotor fl;

    @Hardware(
            name = "FR",
            diameterIn = Constants.Robot.ACTUAL_DIAMETER_IN,
            ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT,
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    )
    public NovelMotor fr;

    @Hardware(
            name = "BL",
            diameterIn = Constants.Robot.ACTUAL_DIAMETER_IN,
            ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT,
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    )
    public NovelMotor bl;

    @Hardware(
            name = "BR",
            diameterIn = Constants.Robot.ACTUAL_DIAMETER_IN,
            ticksPerRevolution = Constants.TickCounts.MOVEMENT_MOTOR_TICK_COUNT,
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    )
    public NovelMotor br;

    @Hardware(name = "imu")
    public IMU imu;

    public IntoTheDeepRobotConfiguration(HardwareMap hardwareMap) {
        super(hardwareMap);

        OctoQuad octoQuad = hardwareMap.get(OctoQuad.class,"OctoQuad");

        this.imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                ))
        );
    }

    @Hardware(name = "OL")
    public NovelMotor odometryLeft;

    @Hardware(name = "OR")
    public NovelMotor odometryRight;

    @Hardware(name = "OP")
    public NovelMotor odometryPerpendicular;

    @Hardware(name = "OctoQuad")
    public OctoQuad octoQuad;


    @Override
    public MecanumDriver createMecanumDriver() {
        return new MecanumDriver(
                this.fl,
                this.fr,
                this.bl,
                this.br,
                Constants.Coefficients.SNOWBALL_COEFFICIENTS
        );
    }

    public OdometryLocalizer createOdometry() {
        return new OdometryLocalizer(
                new OdometryCoefficientSet(1, 1, 1),
                // 4-5-6 is right-left-perpendicular
                new OdometryEncoder(octoQuad,4, Constants.Odometry.ODOMETRY_WHEEL_DIAMETER_IN, Constants.Odometry.TICKS_PER_ODOMETRY_REVOLUTION),
                new OdometryEncoder(octoQuad,5, Constants.Odometry.ODOMETRY_WHEEL_DIAMETER_IN, Constants.Odometry.TICKS_PER_ODOMETRY_REVOLUTION),
                new OdometryEncoder(octoQuad,6, Constants.Odometry.ODOMETRY_WHEEL_DIAMETER_IN, Constants.Odometry.TICKS_PER_ODOMETRY_REVOLUTION),
                9.5,
                11
        );
    }
}