package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimpleMotors {
    // Drivetrain
    public DcMotorEx leftFront, rightFront, leftRear, rightRear;
    
    // Mechanism Motors
    public DcMotorEx shooterMotor0, shooterMotor1;
    public DcMotorEx transferMotor;
    public DcMotorEx intakeMotor;

    public void init(HardwareMap hardwareMap) {
        // Drivetrain - Names from Constants.java
        leftFront = hardwareMap.get(DcMotorEx.class, "ehmotor1");
        rightFront = hardwareMap.get(DcMotorEx.class, "ehmotor2");
        leftRear = hardwareMap.get(DcMotorEx.class, "ehmotor0");
        rightRear = hardwareMap.get(DcMotorEx.class, "ehmotor3");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Mechanism Motors - Names from motors.java
        shooterMotor0 = hardwareMap.get(DcMotorEx.class, "motor0");
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        transferMotor = hardwareMap.get(DcMotorEx.class, "motor2");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "motor3");

        shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Reset encoders for shooter
        shooterMotor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setShooterVelocity(double velocity) {
        shooterMotor0.setVelocity(velocity);
        shooterMotor1.setVelocity(velocity);
    }
}
