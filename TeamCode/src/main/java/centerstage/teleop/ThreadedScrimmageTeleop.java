package centerstage.teleop;

import static centerstage.Constants.ROBOT;

import com.pocolifo.robobase.BuildProperties;
import com.pocolifo.robobase.bootstrap.TeleOpOpMode;
import com.pocolifo.robobase.control.BoolSupplier;
import com.pocolifo.robobase.control.GamepadCarWheels;
import com.pocolifo.robobase.control.Pressable;
import com.pocolifo.robobase.control.Toggleable;
import com.pocolifo.robobase.motor.CarWheels;
import com.pocolifo.robobase.motor.DoubleCRServo;
import com.pocolifo.robobase.motor.DoubleMotor;
import com.pocolifo.robobase.motor.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ScrimmageTeleOp " + BuildProperties.VERSION)
public class ThreadedScrimmageTeleop extends TeleOpOpMode {
    private CarWheels carWheels;
    private GamepadCarWheels gamepadCarWheels;
    private Pressable intake;
    private Motor intakeMotor;
    boolean intakeOpen = false;
    private Thread intakeThread;

    private Pressable LaunchPlane;

    private Pressable outtake;
    private Servo outtakeServo;
    boolean outtakeRunning = false;
    private Thread outtakeThread;

    private DoubleCRServo midjoint;
    private Pressable midjointToTop;
    private Pressable midjointToBottom;
    private Pressable midjointForward;
    private Pressable midjointBackward;
    private Thread midjointThread;

    private DoubleMotor linearSlides;
    private Thread linearSlideThread;
    private Pressable dpadUp;
    private Pressable dpadDown;

    private Pressable fullReset;

    private boolean runThreads = false;
    boolean threadsInitialized = false;

    @Override
    public void initialize() {
        this.carWheels = new CarWheels(
                hardwareMap,
                1120,
                10d,
                ROBOT,
                "FL",
                "FR",
                "BL",
                "BR",
                "FL"
        );
//Gamepad 1

        this.gamepadCarWheels = new GamepadCarWheels(this.carWheels, this.gamepad1, () -> this.gamepad1.a);

        this.intake = new Pressable(() -> this.gamepad1.right_bumper);
        this.intakeMotor = new Motor(hardwareMap.get(DcMotor.class,"SpinningIntake"), 1120);

        this.LaunchPlane = new Pressable(() -> this.gamepad1.b);

//Gamepad 2
        //Todo: may need to tweak coefficients
        linearSlides = new DoubleMotor(new Motor(hardwareMap.get(DcMotor.class, "LeftLinearSlide"), 1120), new Motor(hardwareMap.get(DcMotor.class, "RightLinearSlide"), 1120), 1, -1);
        this.dpadUp = new Pressable(() -> this.gamepad2.dpad_up);
        this.dpadDown = new Pressable(() -> this.gamepad2.dpad_down);

        this.midjoint = new DoubleCRServo(hardwareMap.get(CRServo.class,"rotationLeftServo"), hardwareMap.get(CRServo.class,"rotationRightServo"), 0.5);
        this.midjointToTop = new Pressable(() -> this.gamepad2.x);
        this.midjointToBottom = new Pressable(() -> this.gamepad2.b);
        this.midjointForward = new Pressable(() -> (this.gamepad2.right_trigger > 0.5));
        this.midjointBackward = new Pressable(() -> (this.gamepad2.left_trigger > 0.5));


        this.outtake = new Pressable(() -> this.gamepad2.right_bumper);
        this.outtakeServo = hardwareMap.get(Servo.class, "Outtake");

        System.out.println("Done Initializing");
    }

    //Todo: May also need tweaking
    private void rotateMidjointForward()
    {
        midjoint.spin(DcMotorSimple.Direction.FORWARD);
    }
    private void rotateMidjointBackward()
    {
        midjoint.spin(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    //NOTE: It might be nice to have a processUpdates function that takes an array of togglables and updates all of them
    public void loop() {
        if (!threadsInitialized)
        {
            this.runThreads = true;

            //Gamepad 1
            intakeThread = new Thread(() -> {
                while (runThreads)
                {
                    if (this.intake.get() && !intakeOpen) {
                        intakeMotor.drive(1);
                        intakeOpen = true;
                    }
                    else if (!intake.get() && intakeOpen)
                    {
                        intakeMotor.drive(0);
                        intakeOpen = false;
                    }
                }
            });
            intakeThread.start();
            System.out.println("Intake thread initialized");

            //Gamepad 2
            linearSlideThread = new Thread(() -> {
                while(runThreads) {
                    linearSlides.spin(gamepad2.left_stick_y); //4 is arbitrary, but full power seems like a lot
                    //Or it might use the D-pad:
                    if (dpadUp.get()) {
                        linearSlides.spin(0.75);
                    } else if (dpadDown.get()) {
                        linearSlides.spin(-0.75);
                    } else {
                        linearSlides.stopMoving();
                    }
                }
            });
            linearSlideThread.start();
            System.out.println("Linear slide thread initialized");

            outtakeThread = new Thread(() -> {
                while (runThreads)
                {
                    if (this.outtake.get() && !outtakeRunning) {
                        outtakeServo.setPosition(1);
                        outtakeRunning = true;
                    }
                    else if (!intake.get() && outtakeRunning) {
                        outtakeServo.setPosition(0);
                        outtakeRunning = false;
                    }
                }
            });
            outtakeThread.start();
            System.out.println("Outtake thread initialized");

            midjointThread = new Thread(() -> {
                while(runThreads) {
                    if (midjointToTop.get()) {
                        //Todo: while not at top limit switch
                        rotateMidjointForward();
                        midjoint.stopMoving();
                    } else if (midjointToBottom.get()) {
                        //Todo: while not at bottom limit switch
                        rotateMidjointBackward();
                        midjoint.stopMoving();
                    } else if (midjointForward.get()) {
                        rotateMidjointForward();
                    } else if (midjointBackward.get()) {
                        rotateMidjointBackward();
                    } else {
                        midjoint.stopMoving();
                    }
                }
            });
            midjointThread.start();
            System.out.println("Midjoint thread initialized");

            System.out.println("All threads initialized!");

            threadsInitialized = true;
        }
        //Gamepad 1
            //Driving
               this.gamepadCarWheels.update();
        this.gamepadCarWheels.updateWithDpadDrive();

            //Launch Plane
                if (this.LaunchPlane.get()) {
                    //launch the plane!
                }
            //Hanging?
    }
    @Override
    public void stop()
    {
        runThreads = false;
    }
}
