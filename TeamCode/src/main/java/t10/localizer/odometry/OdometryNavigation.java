package t10.localizer.odometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import t10.motion.mecanum.MecanumDriver;
import t10.geometry.MovementVector;
import t10.motion.profile.TrapezoidalMotionProfile;

public class OdometryNavigation {
    private OdometryLocalizer odometry;
    private MecanumDriver driver;
    public final double minError;
    public final double minAngleError;
    public final double maxLatVelocity;
    public final double maxAngVelocity;

    public OdometryNavigation(OdometryLocalizer odometry, MecanumDriver driver) {
        this.odometry = odometry;
        this.driver = driver;
        this.minError = 0.5;
        this.minAngleError = 2; //in degrees here
        maxLatVelocity = 10;
        maxAngVelocity = 15;
    }


    public void driveLateral(double distance) {
        double initialX = odometry.getFieldCentricPose().getX();
        double finalY = odometry.getFieldCentricPose().getY() + distance;
        while (Math.abs(finalY - odometry.getFieldCentricPose().getY()) > minError) {
            driver.setVelocity(odometry.changeToRobotCenteredVelocity(new MovementVector(10 * Math.signum(distance), 0, 0)));
            this.odometry.update();
        }
        driver.setVelocity(new MovementVector(0, 0, 0));
    }

    public void driveLateralTurn(double distance, TrapezoidalMotionProfile profile, double angleTo) {
        double initialX = odometry.getFieldCentricPose().getX();
        double initialY = odometry.getFieldCentricPose().getY();
        double finalY = odometry.getFieldCentricPose().getY() + distance;
        double lateralVelo = 0;
        double angleVelo = 0;
        while (Math.abs(finalY - odometry.getFieldCentricPose().getY()) > minError) {

            lateralVelo = profile.getVelocity(odometry.getFieldCentricPose().getY() - initialY);

            if (needAngleCorrectionDegrees(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angleTo)) {
                angleVelo =  findTurnSpeed(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angleTo);
            }
            else {
                angleVelo = 0;
            }

            driver.setVelocity(odometry.changeToRobotCenteredVelocity(new MovementVector(
                    lateralVelo,
                    0,
                    angleVelo)));
            this.odometry.update();
        }
        if(needAngleCorrectionDegrees(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angleTo))
        {
            turnAbsolute(angleTo);
        }
        driver.setVelocity(new MovementVector(0, 0, 0));
    }


    public void driveHorizontal(double distance) {
        double initialY = odometry.getFieldCentricPose().getY();
        double initialX = odometry.getFieldCentricPose().getX();
        double finalX = initialX + distance;
        while (Math.abs(finalX - odometry.getFieldCentricPose().getX()) > minError) {
            driver.setVelocity(odometry.changeToRobotCenteredVelocity(new MovementVector(0, 10 * Math.signum(distance), 0)));
            this.odometry.update();
        }
        driver.setVelocity(new MovementVector(0, 0, 0));
    }

    public void driveHorizontalTurn(double distance, TrapezoidalMotionProfile profile, double angleTo) {
        double initialX = odometry.getFieldCentricPose().getX();
        double finalX = odometry.getFieldCentricPose().getX() + distance;
        double horizontalVelo = 0;
        double angleVelo = 0;
        while (Math.abs(finalX - odometry.getFieldCentricPose().getX()) > minError) {

            horizontalVelo = profile.getVelocity(odometry.getFieldCentricPose().getX() - initialX);

            if (needAngleCorrectionDegrees(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angleTo)) {
                angleVelo =  findTurnSpeed(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angleTo);
            }
            else {
                angleVelo = 0;
            }

            driver.setVelocity(odometry.changeToRobotCenteredVelocity(new MovementVector(
                    0,
                    horizontalVelo,
                    angleVelo)));
            this.odometry.update();
        }
        if(needAngleCorrectionDegrees(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angleTo))
        {
            turnAbsolute(angleTo);
        }
        driver.setVelocity(new MovementVector(0, 0, 0));
    }


    public void driveDiagonal(double distanceX, double distanceY) {
        double initialY = odometry.getFieldCentricPose().getY();
        double initialX = odometry.getFieldCentricPose().getX();
        double finalX = initialX + distanceX;
        double finalY = initialY + distanceY;
        while ((Math.abs(finalX - odometry.getFieldCentricPose().getX()) > minError) || (Math.abs(finalY - odometry.getFieldCentricPose().getY()) > minError)) {
            double speedX, speedY;
            speedX = 10 * Math.signum(distanceX);
            speedY = 10 * Math.signum(distanceY);
            driver.setVelocity(odometry.changeToRobotCenteredVelocity(new MovementVector(speedY, speedX, 0)));
            this.odometry.update();
        }
        System.out.println("step 1 done");
        while ((Math.abs(finalX - odometry.getFieldCentricPose().getX()) > minError)) {
            driveHorizontal(finalX - odometry.getFieldCentricPose().getX());
            this.odometry.update();
        }
        while (Math.abs(finalY - odometry.getFieldCentricPose().getY()) > minError) {
            driveLateral(finalY - odometry.getFieldCentricPose().getY());
            this.odometry.update();
        }
        driver.setVelocity(new MovementVector(0, 0, 0));
    }


    public void turnAbsolute(double angle) {
        while (needAngleCorrectionDegrees(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angle))
            while (needAngleCorrectionDegrees(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angle)) {
                driver.setVelocity(new MovementVector(0, 0, findTurnSpeed(odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES), angle)));
                this.odometry.update();
            }
        driver.setVelocity(new MovementVector(0, 0, 0));
    }

    public void turnRelative(double angle) {
        double targetAngle = angle + odometry.getFieldCentricPose().getHeading(AngleUnit.DEGREES);
        if (targetAngle > 180) {
            targetAngle -= 360;
        }
        if (targetAngle < -180) {
            targetAngle += 360;
        }
        turnAbsolute(targetAngle);
    }

    public void driveAbsolute(double targetX, double targetY) {
        double distanceX = targetX - odometry.getFieldCentricPose().getX();
        double distanceY = targetY - odometry.getFieldCentricPose().getY();
        while ((Math.abs(targetX - odometry.getFieldCentricPose().getX()) > minError) || (Math.abs(targetY - odometry.getFieldCentricPose().getY()) > minError)) {
            double speedX, speedY;
            if ((Math.abs(targetX - odometry.getFieldCentricPose().getX()) > minError)) {
                speedX = 10 * Math.signum(distanceX);
            } else {
                speedX = targetX - odometry.getFieldCentricPose().getX();
            }
            if ((Math.abs(targetY - odometry.getFieldCentricPose().getY()) > minError)) {
                speedY = 10 * Math.signum(distanceY);
            } else {
                speedY = targetY - odometry.getFieldCentricPose().getY();
            }
            driver.setVelocity(odometry.changeToRobotCenteredVelocity(new MovementVector(speedY, speedX, 0)));
            this.odometry.update();
        }
        driver.setVelocity(new MovementVector(0, 0, 0));
    }

    /**
     * attempted perfect arbitrary to-point driving
     * public void driveSmart(Pose targetPose)
     * {
     * MovementVector vector = calcTrigVelocity(targetPose,odometry.getFieldCentricPose());
     * vector = new MovementVector(-vector.getVertical(), vector.getHorizontal(), vector.getRotation());
     * driver.setVelocity(vector);
     * }
     * public MovementVector calcTrigVelocity(Pose targetPose, Pose currentPose)
     * {
     * double deltaY = targetPose.getX() - currentPose.getX();
     * double deltaY_abs = Math.abs(deltaY);
     * double deltaX = targetPose.getY() - currentPose.getY();
     * double deltaX_abs = Math.abs(deltaX);
     * double currentAngle = currentPose.getHeading(AngleUnit.RADIANS);
     * double targetAngle = targetPose.getHeading(AngleUnit.RADIANS);
     * if (deltaY_abs < minError && deltaX_abs < minError)
     * {
     * return new MovementVector(0,0, findTurnSpeed(currentAngle,targetAngle));
     * }
     * else if (deltaY_abs < 5*minError && deltaX_abs > 5*minError)
     * {
     * return odometry.getRobotCentricVelocity(10*Math.signum(deltaY),deltaX);
     * <p>
     * }
     * else if (deltaY_abs > 5*minError && deltaX_abs < 5*minError)
     * {
     * return odometry.getRobotCentricVelocity(deltaY,10 * Math.signum(deltaX));
     * }
     * else if (deltaX_abs > 5*minError && deltaY_abs > 5*minError)
     * {
     * return odometry.getRobotCentricVelocity(10 * Math.signum(deltaY),10 * Math.signum(deltaX));
     * }
     * else {
     * return new MovementVector(deltaY,deltaX,0);
     * }
     * }
     */

    public boolean needAngleCorrectionDegrees(double currentAngle, double targetAngle) {
        double startAngle = currentAngle + 180;
        double endAngle = targetAngle + 180;
        if ((startAngle < minAngleError && endAngle > 360 - startAngle) || (endAngle < minAngleError && startAngle > 360 - endAngle)) {
            return false;
        } else if (Math.abs(endAngle - startAngle) < minAngleError) {
            return false;
        } else {
            return true;
        }
    }


    public double findTurnSpeed(double currentAngle, double targetAngle) {
        double direction = 0;
        if (Math.abs(targetAngle) == 180) {
            targetAngle = 180 * Math.signum(currentAngle);
        }
        if (targetAngle < currentAngle - 180) {
            direction = 1;
        } else if (targetAngle > currentAngle + 180) {
            direction = -1;
        } else if (targetAngle < currentAngle) {
            direction = -1;
        } else if (targetAngle > currentAngle) {
            direction = 1;
        }

        if (Math.abs(targetAngle - currentAngle) < 5 * minAngleError) {
            return direction * Math.abs(targetAngle - currentAngle);
        } else if (Math.abs(targetAngle - currentAngle) > 360 - 5 * minAngleError) {
            return direction * Math.abs(360 - (Math.abs(targetAngle - currentAngle)));
        } else {
            return maxAngVelocity * direction;
        }
    }
}