package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class motors {

    // ---- Constants ----
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;

    // ---- Motor Vars ----
    private DcMotorEx shooterMotor1, shooterMotor0, transferMotor, intakeMotor, m0,m1,m2,m3;

    // ---- State ----
    private boolean isIntakeOn = false;
    private boolean isTransferOn = false;

    // ---- Constructor ----
    public motors() {

    }


    /**
     * Initializes all motors and sets their initial states.
     */
    public void init(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        shooterMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        transferMotor = hardwareMap.get(DcMotorEx.class, "motor2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "motor3");

        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Set initial power to 0
        setShooterVelocity(0);
        setTransferPower(0);
        setIntakePower(0);
    }

    public boolean isIntakeOn() {
        return this.isIntakeOn;
    }

    public boolean isTransferOn() {
        return this.isTransferOn;
    }


    /**
     * Activates the intake and transfer motors to shoot a note.
     */
    public void shoot() {
        setIntakePower(INTAKE_POWER);
        setTransferPower(TRANSFER_POWER);
        isIntakeOn = true;
        isTransferOn = true;

    }

    public boolean setIntake(boolean on){
        if(on){
            setIntakePower(INTAKE_POWER);
            isIntakeOn = true;
            return true;
        }else{
            setIntakePower(0);
            isIntakeOn = false;
            return false;
        }
    }

    public boolean setTransfer(boolean on){
        if(on){
            setTransferPower(TRANSFER_POWER);
            isTransferOn = true;
            return true;
        }else{
            setTransferPower(0);
            isTransferOn = false;
            return false;
        }
    }

    /**
     * Toggles the intake motor on or off. Manages transfer motor for smooth pixel transition.
     */
    public void toggleIntake() {
        if (!isIntakeOn) {
            // Turn intake on
            setIntakePower(INTAKE_POWER);
            setTransferPower(-0.1);
            isIntakeOn = true;
            isTransferOn = false;
        } else {
            // Turn intake off
            setIntakePower(0.0); // Keep a slight forward power to settle pixels
            setTransferPower(0);
            isIntakeOn = false;
            isTransferOn = false;
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

    public void stopTransfer(){
        transferMotor.setPower(0);
    }

    // --- Low-level motor control --- //

    public void setShooterVelocity(double velocity) {
        if (shooterMotor0 != null) {
            shooterMotor0.setVelocity(velocity);
        }
        if (shooterMotor1 != null) {
            shooterMotor1.setVelocity(velocity);
        }
    }

    public void setTransferPower(double power) {
        if (transferMotor != null) transferMotor.setPower(power);
    }

    public void setIntakePower(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
    }


}
