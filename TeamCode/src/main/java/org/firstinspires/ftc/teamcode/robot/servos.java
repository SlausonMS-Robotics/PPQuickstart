package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class servos {

    // ---- Constants ----

    public static final double TURRET_MIN_POS = .3;
    public static final double TURRET_MAX_POS = .7;
    public static final double MANUAL_TURRET_SPEED = 0.005; // Adjust this for manual control sensitivity

    // ---- PID Constants ----
    // NOTE: These will need to be tuned for your specific robot
    public static final double TURRET_P = 0.5;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0;

    // ---- PID Controller ----
    private other_helpers pidController = new other_helpers();

    // ---- Servos ----
    private ServoImplEx turretServo;     // servo0
    private ServoImplEx indexerServo;  // servo1
    private ServoImplEx ledServo;      // servo2 for LED control

    public enum LedColor { GREEN, RED, OFF }

    /**
     * Initializes all servos and the PID controller for the turret.
     */
    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, "servo0");
        indexerServo = hardwareMap.get(ServoImplEx.class, "servo1");
        ledServo = hardwareMap.get(ServoImplEx.class, "led");
        ledServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Center the turret on initialization
        setTurretServoPos(0.5);

        // Initialize the PID controller with our constants
        pidController.initPID(TURRET_P, TURRET_I, TURRET_D);
    }

    public void setLedColor(LedColor color) {
        switch (color) {
            case GREEN:
                ledServo.setPosition(1.0);
                break;
            case RED:
                ledServo.setPosition(0.0);
                break;
            case OFF:
            default:
                ledServo.setPosition(0.5);
                break;
        }
    }

    /**
     * Manually moves the turret based on a power value.
     * @param power The desired power, from -1.0 (left) to 1.0 (right).
     */
    public void moveTurretManually(double power) {
        double currentPos = getTurretServoPos();
        double newPos = currentPos + (power * MANUAL_TURRET_SPEED);
        setTurretServoPos(newPos);
    }

    /**
     * Updates the turret position using a PID controller to minimize heading error.
     * @param headingError The error in radians between the current and target heading.
     */
    public void updateTurretWithPID(double headingError) {
        double pidCorrection = pidController.updatePID(headingError, 0);
        double currentPos = getTurretServoPos();
        double newPos = currentPos + pidCorrection;
        setTurretServoPos(newPos);
    }

    public void setTurretServoPos(double pos) {
        if (turretServo != null) {
            turretServo.setPosition(Range.clip(pos, TURRET_MIN_POS, TURRET_MAX_POS));
        }
    }

    public double getTurretServoPos() {
        if (turretServo != null) {
            return turretServo.getPosition();
        }
        return 0.5; // Default to center
    }

    public void setIndexerServoPos(double pos) {
        if (indexerServo != null) indexerServo.setPosition(pos);
    }

}
