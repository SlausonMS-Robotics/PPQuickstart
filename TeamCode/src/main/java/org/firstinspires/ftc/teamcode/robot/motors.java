package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class motors {

    // ---- Constants ----
    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 1.0;
    private static final double TRANSFER_REVERSE_POWER = -.18;
    public int transferState = 0;
    public int intakeState = 0;
    public int shooterState = 0;

    // ---- Motor Fields ----
    private DcMotorEx shooterMotor0;
    private DcMotorEx shooterMotor1;
    private DcMotorEx transferMotor;
    private DcMotorEx intakeMotor;
    private static final int ticks_per_rev = 28;


    public motors() {}

    /**
     * Initializes all motors and assigns them to their fields.
     */
    public void init(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        shooterMotor0.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, "motor2");
        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "motor3");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set initial power to 0 for all motors
        setShooterVelocity(0);
        setTransferPower(0);
        setIntakePower(0);
    }

    // ---- Specific Action Methods ----

    public void shoot() {
        setIntakeState(1);
        setTransferState(1);
    }

    public double getShooterVelocity(int motorRPM){
        return motorRPM * ticks_per_rev / 60;
    }



    public int getTransferState(){
        return transferState;
    }

    public int getIntakeState(){
        return intakeState;
    }

    public int getShooterState(){
        return shooterState;
    }

    public void setIntakeState(int state) {
        switch (state) {
            case 1:
                setIntakePower(INTAKE_POWER);
                transferState = 1;
                break;
            case 0:
                setIntakePower(0);
                transferState = 0;
                break;
        }
    }

    public void setTransferState(int state) {
        switch (state) {
            case 1:
                setTransferPower(TRANSFER_POWER);
                transferState = 1;
                break;
            case 0:
                setTransferPower(0);
                transferState = 0;
                break;
            case 2:
                setTransferPower(TRANSFER_REVERSE_POWER);
                transferState = 2;
                break;
        }
    }





    public void setShooterVelocity(double velocity) {
        if (shooterMotor0 != null) shooterMotor0.setVelocity(velocity);
        if (shooterMotor1 != null) shooterMotor1.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        if (shooterMotor0 != null) {
            return shooterMotor0.getVelocity();
        }
        return 0;
    }

    public void setTransferPower(double power) {
        if (transferMotor != null) {
            transferMotor.setPower(power);
        }
    }

    public double getTransferPower() {
        if (transferMotor != null) {
            return transferMotor.getPower();
        }
        return 0;
    }

    public void setIntakePower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    public double getIntakePower() {
        if (intakeMotor != null) {
            return intakeMotor.getPower();
        }
        return 0;
    }
}
