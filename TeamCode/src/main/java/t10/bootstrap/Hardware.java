package t10.bootstrap;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import com.qualcomm.robotcore.hardware.DcMotor;

import t10.motion.hardware.Motor;

/**
 * <p>
 * {@code @Hardware} annotations are used in robot configurations (see {@link AbstractRobotConfiguration}). In the following example, {@code webcam} will be set to the {@link t10.vision.Webcam} found in the hardware map with name "Webcam".
 * <pre>{@code
 * @Hardware(name = "Webcam")
 * public Webcam webcam;
 * }</pre>
 * </p>
 * <p>
 * The following example sets {@code chainWheel} to a {@link Motor} found in the hardware map at "Chain". It also configures
 * <ul>
 *     <li>the wheel diameter to be 9.6cm when initializing the Motor</li>
 *     <li>the ticks per revolution to 500</li>
 *     <li>the zero power behavior to brake</li>
 * </ul>
 * <pre>{@code
 *      @Hardware(
 *          name = "Chain",
 *          wheelDiameterCm = 9.6,
 *          ticksPerRevolution = 500,
 *          zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
 *      )
 *      public Motor chainWheel;
 * }</pre>
 * </p>
 * <p><strong>Note that the types of the variables correspond to the type of hardware found in the hardware map. Chain is a motor, so Motor can be used. It cannot be used for webcam, because "Webcam" does not point to a motor.</strong></p>
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface Hardware {
	/**
	 * The name of the hardware in the hardware map.
	 */
	String name();

	// Optional parameters just for certain types of hardware

	/**
	 * For motors only. The number of encoder ticks per revolution.
	 */
	double ticksPerRevolution() default -1;

	/**
	 * For motors only. The zero power behavior of the motor.
	 */
	DcMotor.ZeroPowerBehavior zeroPowerBehavior() default DcMotor.ZeroPowerBehavior.BRAKE;

	/**
	 * For motors only. The gear ratio of the motor.
	 */
	int gearRatio() default 1;

	/**
	 * For wheels only. The diameter of the wheel.
	 */
	double diameterIn() default -1;
}
