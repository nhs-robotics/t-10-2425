package intothedeep.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import intothedeep.capabilities.ArmCapabilities;
import intothedeep.capabilities.ArmExtensionCapabilities;
import intothedeep.capabilities.ClawCapabilities;
import intothedeep.capabilities.CraneCapabilities;
import intothedeep.SnowballConfig;
import t10.bootstrap.TeleOpOpMode;
import t10.gamepad.GController;
import t10.localizer.odometry.OdometryLocalizer;
import t10.motion.mecanum.MecanumDriver;

@TeleOp
public class CompetitionTeleOp extends TeleOpOpMode {
    protected SnowballConfig config;
    protected CraneCapabilities crane;
    private ArmCapabilities arm;
    protected GController g1;
    protected GController g2;
    private MecanumDriver driver;
    private OdometryLocalizer odometry;
    private ClawCapabilities claw;
    private ArmExtensionCapabilities extension;

    private Telemetry.Item x;
    private Telemetry.Item y;
    private Telemetry.Item r;
    private Telemetry.Item extend_length, rotation, craneLeft, craneRight;

    @Override
    public void initialize() {
        this.config = new SnowballConfig(this.hardwareMap);
        this.crane = new CraneCapabilities(this.config);
        this.arm = new ArmCapabilities(this.config);
        this.claw = new ClawCapabilities(this.config);
        this.extension = new ArmExtensionCapabilities(config);
        this.g2 = new GController(this.gamepad2)
                .dpadUp.onPress(() -> this.extension.extend(1)).onRelease(() -> this.extension.extend(0)).ok()
                .dpadDown.onPress(() -> this.extension.extend(-1)).onRelease(() -> this.extension.extend(0)).ok()
                .a.onPress(() -> this.claw.toggle()).ok()
                .x.onPress(() -> this.crane.positionHighBasket()).ok();
        this.g1 = new GController(this.gamepad1)
                .x.initialToggleState(true).ok();
        this.driver = this.config.createMecanumDriver();
        this.odometry = config.createOdometry();

        this.x = this.telemetry.addData("x_novel: ", "0");
        this.y = this.telemetry.addData("y_novel: ", "0");
        this.r = this.telemetry.addData("r_novel: ", "0");
        this.extend_length = this.telemetry.addData("Extension: ", 0);
        this.rotation = this.telemetry.addData("Rotation: ", 0);
        this.craneLeft = this.telemetry.addData("Crane Left: ", 0);
        this.craneRight = this.telemetry.addData("Crane Right: ", 0);
    }

    @Override
    public void loop() {
        if (Math.abs(this.gamepad2.left_stick_y) < 0.1) {
            this.arm.rotate(0);
        } else {
            this.arm.rotate(this.gamepad2.left_stick_y);
        }

        if (Math.abs(this.gamepad2.right_stick_y) < 0.1) {
            this.crane.runCrane(0);
        } else {
            this.crane.runCrane(this.gamepad2.right_stick_y);
        }

        this.x.setValue(this.odometry.getFieldCentricPose().getX());
        this.y.setValue(this.odometry.getFieldCentricPose().getY());
        this.r.setValue(this.odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES));
//        this.extend_length.setValue(config.armExtension.motor.getCurrentPosition());
//        this.rotation.setValue(config.armRotation.motor.getCurrentPosition());
        this.extend_length.setValue(config.liftLeft.motor.getCurrentPosition());
        this.rotation.setValue(crane.position);
        this.craneLeft.setValue(config.liftLeft.motor.getPower());
        this.craneRight.setValue(config.liftRight.motor.getPower());

        this.telemetry.update();
        this.odometry.update();
        this.driver.useGamepad(this.gamepad1, this.g1.x.isToggled() ? 1 : 0.5);
        this.g2.update();
        this.g1.update();
        this.crane.update();
        this.arm.update();
    }
}
