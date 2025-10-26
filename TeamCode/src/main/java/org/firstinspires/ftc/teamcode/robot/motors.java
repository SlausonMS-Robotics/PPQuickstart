package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class motors {

    private DcMotorEx shooterMotor1, shooterMotor0; // formerly motor0

    /**
     * Initializes the lift motor.
     * Add other motors here as needed.
     */
    public void init(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "ehmotor1");
        shooterMotor0 = hardwareMap.get(DcMotorEx.class, "ehmotor0");
        shooterMotor0.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void shooterPower(double pow){
        shooterMotor0.setPower(pow);
        shooterMotor1.setPower(pow);
    }

    /**
     * Sets the lift motor target position and velocity.
     */
    public void setLiftMotor(int pos, int vel) {
        if (shooterMotor1 == null) return;

        shooterMotor1.setTargetPosition(pos);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setVelocity(vel);
    }

    /**
     * Stops the lift motor.
     */
    public void stopLiftMotor() {
        if (shooterMotor1 != null) {
            shooterMotor1.setPower(0);
        }
    }

    /**
     * Sets the lift motor's target position tolerance.
     */
    public void setLiftMotorPosTolerance(int tolerance) {
        if (shooterMotor1 != null) {
            shooterMotor1.setTargetPositionTolerance(tolerance);
        }
    }

    /**
     * Enables RUN_TO_POSITION mode for lift motor.
     */
    public void runToPosition(int pos, double power) {
        if (shooterMotor1 == null) return;

        shooterMotor1.setTargetPosition(pos);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shooterMotor1.setPower(power);
    }

    /**
     * Checks if lift motor is still moving toward target.
     */
    public boolean isLiftBusy() {
        return shooterMotor1 != null && shooterMotor1.isBusy();
    }

    /**
     * Gets the current position of the lift motor.
     */
    public int getLiftPosition() {
        return shooterMotor1 != null ? shooterMotor1.getCurrentPosition() : 0;
    }

    /**
     * Sets lift motor to brake or float when power is zero.
     */
    public void setLiftZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        if (shooterMotor1 != null) {
            shooterMotor1.setZeroPowerBehavior(behavior);
        }
    }
}
