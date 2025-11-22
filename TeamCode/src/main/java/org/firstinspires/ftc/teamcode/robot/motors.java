package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class motors {

    // ---- Constants ----
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;

    // ---- Motor Vars ----
    private DcMotorEx shooterMotor1, shooterMotor0, transferMotor, intakeMotor;

    // ---- Helper Classes ----
    private servos myServos; // Reference to the servos class

    // ---- State ----
    private boolean isIntakeOn = false;
    private boolean isTransferOn = false;

    /**
     * Initializes all motors and sets their initial states.
     */
    public void init(HardwareMap hardwareMap, servos myServos) {
        this.myServos = myServos; // Store the servos object

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
        setShooterVelocity(4200);
        setTransferPower(0);
        setIntakePower(0);
    }

    // ---- Public Getters for State and Motors ----
    public boolean isIntakeOn() {
        return this.isIntakeOn;
    }

    public boolean isTransferOn() {
        return this.isTransferOn;
    }

    public DcMotorEx getShooterMotor1() { return shooterMotor1; }
    public DcMotorEx getShooterMotor0() { return shooterMotor0; }
    public DcMotorEx getTransferMotor() { return transferMotor; }
    public DcMotorEx getIntakeMotor() { return intakeMotor; }


    /**
     * Gets the current draw of a specific motor in Amps.
     * @param motor The DcMotorEx object to measure.
     * @return The current in Amps.
     */
    public double getMotorCurrent(DcMotorEx motor){
        if (motor == null) return 0;
        return motor.getCurrent(CurrentUnit.AMPS);
    }


    /**
     * Activates the intake and transfer motors to shootAction a note.
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
            myServos.setIntakeServos(true); // Control the intake servos
            isIntakeOn = true;
            return true;
        }else{
            setIntakePower(0);
            myServos.setIntakeServos(false); // Control the intake servos
            isIntakeOn = false;
            return false;
        }
    }

    public void transferBackwards(){
        setTransferPower(-0.17);
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
        isIntakeOn = !isIntakeOn; // Toggle the state
        myServos.setIntakeServos(isIntakeOn);

        if (isIntakeOn) {
            // Turn intake on
            setIntakePower(INTAKE_POWER);
            transferBackwards();
            isTransferOn = false;
        } else {
            // Turn intake off
            setIntakePower(0.0);
            setTransferPower(0);
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
        if (transferMotor != null) {
            transferMotor.setPower(power);
        }
    }

    public void setIntakePower(double power) {
        if (intakeMotor != null) intakeMotor.setPower(power);
    }


}
