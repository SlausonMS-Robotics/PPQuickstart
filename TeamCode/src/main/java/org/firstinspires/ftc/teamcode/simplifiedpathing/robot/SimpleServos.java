package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class SimpleServos {
    public ServoImplEx turretServo, ledServo, bouncerServo;
    public Servo intakeServo1, intakeServo2, intakeServo3;

    public enum LedColor { GREEN, RED, VIOLET, YELLOW, OFF }

    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, "shservo0");
        ledServo = hardwareMap.get(ServoImplEx.class, "shservo1");
        bouncerServo = hardwareMap.get(ServoImplEx.class, "shservo3");
        
        intakeServo1 = hardwareMap.get(Servo.class, "shservo4");
        intakeServo2 = hardwareMap.get(Servo.class, "shservo5");
        intakeServo3 = hardwareMap.get(Servo.class, "shservo2");

        ledServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void setLedColor(LedColor color) {
        switch (color) {
            case GREEN:  ledServo.setPosition(0.5);  break;
            case RED:    ledServo.setPosition(0.28); break;
            case VIOLET: ledServo.setPosition(0.72); break;
            case YELLOW: ledServo.setPosition(0.39); break;
            case OFF:    ledServo.setPosition(0.0);  break;
        }
    }

    public void setIntakeServos(boolean on) {
        if (on) {
            intakeServo1.setPosition(1.0);
            intakeServo2.setPosition(0.0);
            intakeServo3.setPosition(1.0);
        } else {
            intakeServo1.setPosition(0.5);
            intakeServo2.setPosition(0.5);
            intakeServo3.setPosition(0.5);
        }
    }
}
