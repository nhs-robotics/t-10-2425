package intothedeep;

import t10.utils.PIDController;


/**
 * Rotation:
 * - Power > 0
 *      - Rotate UPWARDS
 *      - Position (Ticks) INCREASES
 * - Power < 0
 *      - Rotate DOWNWARDS
 *      - Position (Ticks) DECREASES
 */
public class ArmCapabilities {
    public static final int HORIZONTAL_TICKS = 0;
    public static final int INSPECTION_TICKS = 775;
    public static final int MIN_ROTATION = -50; //TODO: find better value
    public static final int MAX_ROTATION = 788; //Fully Up
    private final SnowballConfig config;
    private final PIDController armRotationStabilizer;
    private int armRotationTarget;
    private boolean isRotationStabilizerEnabled = true;
    private int armRotationPosition;

    public ArmCapabilities(SnowballConfig configuration) {
        this.config = configuration;
        this.armRotationStabilizer = new PIDController(0.05, 0, 0);
    }

    public void update() {
         this.armRotationPosition = this.config.armRotation.motor.getCurrentPosition();

        if (this.isRotationStabilizerEnabled) {
            double powerForRotationMotor = this.armRotationStabilizer.calculate(
                    this.armRotationPosition,
                    this.armRotationTarget
            );

            this.rotate(powerForRotationMotor);
        }

        if (!this.isExtensionAllowed(this.config.armExtension.motor.getPower())) {
            this.extend(0);
        }
    }

    public void extend(double power) {
        this.config.armExtension.setPower(power);
    }

    public void rotate(double power) {
        if (this.isRotationAllowed(power)) {
            this.config.armRotation.motor.setPower(0);
            this.config.armRotation.setPower(power);
        }

        if (power == 0) {
            this.isRotationStabilizerEnabled = true;
            this.armRotationTarget = this.config.armRotation.motor.getCurrentPosition();
        } else {
            this.isRotationStabilizerEnabled = false;
        }
    }

    public void setRotationPosition(int position) {
        this.armRotationTarget = position;
        this.isRotationStabilizerEnabled = true;
    }

    private boolean isExtensionAllowed(double power) {
        int currentPosition = config.armExtension.motor.getCurrentPosition();
        boolean isOutsideBounds = (currentPosition < 0 && power < 0) || (currentPosition > 0 && power > 0);

        return !isOutsideBounds;
    }

    private boolean isRotationOutsideBounds() {
        return this.armRotationPosition < MIN_ROTATION || this.armRotationPosition > MAX_ROTATION;
    }

    private boolean isRotationAllowed(double power) {
//        return this.armRotationPosition > MIN_ROTATION && power > 0


//        if (this.armRotationPosition > MIN_ROTATION) {
//
//        } else if (this.armRotationPosition < MIN_ROTATION) {
//            if (power > 0) {
//                return true;
//            } else if (power < 0) {
//                return false;
//            }
//        } else if (this.armRotationPosition > MAX_ROTATION) {
//
//        }

        return (this.armRotationPosition > MIN_ROTATION && power > 0) || (this.armRotationPosition > MAX_ROTATION && power > 0);
    }
}
