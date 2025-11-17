package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class motors {

    // ---- Constants ----
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;

    // ---- Motor Vars ----
    private DcMotorEx shooterMotor; // Expansion Hub port 0
    private DcMotorEx transferMotor;  // Expansion Hub port 1
    private DcMotorEx intakeMotor;    // Expansion Hub port 2

    // ---- State ----
    private boolean isIntakeOn = false;

    /**
     * Initializes all motors and sets their initial states.
     */
    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        // Set initial power to 0
        setShooterVelocity(0);
        setTransferPower(0);
        setIntakePower(0);
    }

    /**
     * Activates the intake and transfer motors to shoot a note.
     */
    public void shoot() {
        setIntakePower(INTAKE_POWER);
        setTransferPower(TRANSFER_POWER);
        isIntakeOn = true;
    }

    /**
     * Toggles the intake motor on or off. Manages transfer motor for smooth pixel transition.
     */
    public void toggleIntake() {
        if (!isIntakeOn) {
            // Turn intake on
            setIntakePower(INTAKE_POWER);
            setTransferPower(-0.05); // Briefly reverse transfer to prevent jams
            isIntakeOn = true;
        } else {
            // Turn intake off
            setIntakePower(0.05); // Keep a slight forward power to settle pixels
            setTransferPower(0);
            isIntakeOn = false;
        }
    }

    /**
     * Stops all intake and transfer motors.
     */
    public void stopIntakeAndTransfer() {
        setIntakePower(0);
        setTransferPower(0);
        isIntakeOn = false;
    }

    // --- Low-level motor control --- //

    public void setShooterVelocity(double velocity) {
        if (shooterMotor != null) shooterMotor.setVelocity(velocity);
    }

    public void setTransferPower(double power) {
        if (transferMotor != null) transferMotor.setPower(power);
    }

    public void setIntakePower(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
    }
}
