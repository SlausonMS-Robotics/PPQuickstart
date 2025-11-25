package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class servos {

    // ---- Constants ----

    public static final double TURRET_MIN_POS = .3;
    public static final double TURRET_MAX_POS = .7;

    // ---- PID Constants ----
    // NOTE: These will need to be tuned for your specific robot
    public static final double TURRET_P = 0.9;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.3;

    // ---- PID Controller ----
    private other_helpers pidController = new other_helpers();

    // ---- Servos ----
    private ServoImplEx turretServo, ledServo, intakeServo1, intakeServo2;


    public enum LedColor { GREEN, RED, VIOLET, OFF }

    /**
     * Initializes all servos and the PID controller for the turret.
     */
    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, "shservo0");
        ledServo = hardwareMap.get(ServoImplEx.class, "shservo1");
        intakeServo1 = hardwareMap.get(ServoImplEx.class, "shservo3");
        intakeServo2 = hardwareMap.get(ServoImplEx.class, "shservo4");

        ledServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Center the turret on initialization
        //setTurretServoPos(0.5);

        // Initialize the PID controller with our constants
        pidController.initPID(TURRET_P, TURRET_I, TURRET_D);
    }

    public void setLedColor(LedColor color) {
        switch (color) {
            case GREEN:
                ledServo.setPosition(.5);
                break;
            case RED:
                ledServo.setPosition(0.28);
                break;
            case VIOLET:
                ledServo.setPosition(0.72);
                break;
            case OFF:
            default:
                ledServo.setPosition(0.0);
                break;
        }
    }

    /**
     * Manually moves the turret based on a power value.
     * @param increment the distance to move the turret in one call of the method.
     */
    public void moveTurretManually(double increment) {
        double currentPos = getTurretServoPos();
        double newPos = currentPos + increment;
        setTurretServoPos(newPos);
    }

    public void setIntakeServos(boolean on){
        if(on){
            intakeServo1.setPosition(1);
            intakeServo2.setPosition(0);
        }
        else{
            intakeServo1.setPosition(.5);
            intakeServo2.setPosition(.5);
        }
    }

    /**
     * Updates the turret position using a PID controller to minimize heading error.
     * @param headingErrorDeg The error in radians between the current and target heading.
     */
    public void updateTurretWithPID(double headingErrorDeg) {
        double pidCorrection = pidController.updatePID(headingErrorDeg, 0);
        double currentPos = getTurretServoPos();
        double newPos = currentPos + pidCorrection;
        setTurretServoPos(newPos);
    }

    public void setTurretServoPos(double pos) {
        turretServo.setPosition(Range.clip(pos, TURRET_MIN_POS, TURRET_MAX_POS));
    }

    public double getTurretServoPos() {
        return turretServo.getPosition();
    }





}
