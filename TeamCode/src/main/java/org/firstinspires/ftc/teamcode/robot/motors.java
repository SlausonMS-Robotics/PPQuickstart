package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class motors {

    // ---- Constants ----
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;
    private static final String SHOOTER_0_NAME = "shooter0";
    private static final String SHOOTER_1_NAME = "shooter1";
    private static final String TRANSFER_NAME = "transfer0";
    private static final String INTAKE_NAME = "intake0";

    // ---- Motor Storage ----
    private Map<String, DcMotorEx> motorMap = new HashMap<>();

    // ---- State ----
    private boolean isIntakeOn = false;
    private boolean isTransferOn = false;

    public motors() {}

    /**
     * Initializes all motors and stores them in a map for easy access.
     */
    public void init(HardwareMap hardwareMap) {
        // Create and initialize each motor, then add it to the map.
        DcMotorEx shooterMotor1 = hardwareMap.get(DcMotorEx.class, SHOOTER_1_NAME);
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorMap.put(SHOOTER_1_NAME, shooterMotor1);

        DcMotorEx shooterMotor0 = hardwareMap.get(DcMotorEx.class, SHOOTER_0_NAME);
        shooterMotor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorMap.put(SHOOTER_0_NAME, shooterMotor0);

        DcMotorEx transferMotor = hardwareMap.get(DcMotorEx.class, TRANSFER_NAME);
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorMap.put(TRANSFER_NAME, transferMotor);

        DcMotorEx intakeMotor = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        motorMap.put(INTAKE_NAME, intakeMotor);

        // Set initial power to 0 for all motors
        setShooterVelocity(0);
        setTransferPower(0);
        setIntakePower(0);
    }

    // ---- Generic Getters and Setters ----

    public DcMotorEx getMotor(String motorName) {
        return motorMap.get(motorName);
    }

    public void setMotorVelocity(String motorName, double velocity) {
        DcMotorEx motor = motorMap.get(motorName);
        if (motor != null) {
            motor.setVelocity(velocity);
        }
    }

    public double getMotorVelocity(String motorName) {
        DcMotorEx motor = motorMap.get(motorName);
        if (motor != null) {
            return motor.getVelocity();
        }
        return 0;
    }

    public void setMotorPower(String motorName, double power) {
        DcMotorEx motor = motorMap.get(motorName);
        if (motor != null) {
            motor.setPower(power);
        }
    }

    public double getMotorPower(String motorName) {
        DcMotorEx motor = motorMap.get(motorName);
        if (motor != null) {
            return motor.getPower();
        }
        return 0;
    }

    // ---- Specific Action Methods ----

    public void shoot() {
        setIntakePower(INTAKE_POWER);
        setTransferPower(TRANSFER_POWER);
        isIntakeOn = true;
        isTransferOn = true;
    }

    /**
     * Toggles the intake state between on and off.
     */
    public void toggleIntake() {
        // Call the setter with the opposite of the current state.
        setIntakeState(!isIntakeOn);
    }

    /**
     * Explicitly sets the intake mechanism to an on or off state.
     * This is an overloaded method that provides the "optional" parameter.
     * @param on True to turn the intake on, false to turn it off.
     */
    public void setIntakeState(boolean on) {
        if (on) {
            // Logic to turn the intake on
            setIntakePower(INTAKE_POWER);
            setTransferPower(-0.18); // Anti-creep
            isIntakeOn = true;
            isTransferOn = false;
        } else {
            // Logic to turn the intake off
            setIntakePower(0.0);
            setTransferPower(0);
            isIntakeOn = false;
            isTransferOn = false;
        }
    }

    // --- Specific Setters (for backward compatibility) --- //

    public void setShooterVelocity(double velocity) {
        setMotorVelocity(SHOOTER_0_NAME, velocity);
        setMotorVelocity(SHOOTER_1_NAME, velocity);
    }

    public void setTransferPower(double power) {
        setMotorPower(TRANSFER_NAME, power);
    }

    public void setIntakePower(double power) {
        setMotorPower(INTAKE_NAME, power);
    }

    // --- Specific Getters --- //

    public double getShooterVelocity() {
        return getMotorVelocity(SHOOTER_0_NAME);
    }

    public double getTransferPower() {
        return getMotorPower(TRANSFER_NAME);
    }

    public double getIntakePower() {
        return getMotorPower(INTAKE_NAME);
    }
}
