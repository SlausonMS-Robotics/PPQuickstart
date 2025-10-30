package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class motors {

    private DcMotorEx shooterMotor1, shooterMotor0;

    /**
     * Initializes the shooter motors.
     * Add other motors here as needed.
     */
    public void init(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        shooterMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        shooterMotor0.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void shooterPower(double pow){
        shooterMotor0.setPower(pow);
        shooterMotor1.setPower(pow);
    }

    /**
     * Sets the PIDF coefficients for the shooter motors.
     * @param pidfCoefficients The PIDF coefficients to set.
     */
    public void setShooterPIDFCoefficients(PIDFCoefficients pidfCoefficients) {
        if (shooterMotor0 != null) {
            shooterMotor0.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
        if (shooterMotor1 != null) {
            shooterMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        }
    }

    /**
     * Sets the shooter motors target velocity.
     */
    public void setShooterVelocity(double velocity) {
        if (shooterMotor0 != null) {
            shooterMotor0.setVelocity(velocity);
        }
        if (shooterMotor1 != null) {
            shooterMotor1.setVelocity(velocity);
        }
    }

    /**
     * Stops the shooter motors.
     */
    public void stopShooterMotors() {
        if (shooterMotor0 != null) {
            shooterMotor0.setPower(0);
        }
        if (shooterMotor1 != null) {
            shooterMotor1.setPower(0);
        }
    }

    /**
     * Sets the shooter motors' target position tolerance.
     */
    public void setShooterMotorPosTolerance(int tolerance) {
        if (shooterMotor0 != null) {
            shooterMotor0.setTargetPositionTolerance(tolerance);
        }
        if (shooterMotor1 != null) {
            shooterMotor1.setTargetPositionTolerance(tolerance);
        }
    }

    /**
     * Enables RUN_TO_POSITION mode for shooter motors.
     */
    public void runToPosition(int pos, double power) {
        if (shooterMotor0 == null || shooterMotor1 == null) return;

        shooterMotor0.setTargetPosition(pos);
        shooterMotor1.setTargetPosition(pos);
        shooterMotor0.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shooterMotor0.setPower(power);
        shooterMotor1.setPower(power);
    }

    /**
     * Checks if shooter motors are still moving toward target.
     */
    public boolean areShooterMotorsBusy() {
        return (shooterMotor0 != null && shooterMotor0.isBusy()) || (shooterMotor1 != null && shooterMotor1.isBusy());
    }

    /**
     * Gets the current position of the shooter motor.
     * This will return the position of shooterMotor0.
     */
    public int getShooterPosition() {
        return shooterMotor0 != null ? shooterMotor0.getCurrentPosition() : 0;
    }

    /**
     * Sets shooter motors to brake or float when power is zero.
     */
    public void setShooterZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        if (shooterMotor0 != null) {
            shooterMotor0.setZeroPowerBehavior(behavior);
        }
        if (shooterMotor1 != null) {
            shooterMotor1.setZeroPowerBehavior(behavior);
        }
    }
}
