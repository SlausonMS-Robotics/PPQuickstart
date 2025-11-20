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
    public static final double TURRET_P = 0.015;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.005;

    // ---- PID Controller ----
    private other_helpers pidController = new other_helpers();

    // ---- Servos ----
    private ServoImplEx turretServo;     // servo0
    private ServoImplEx indexerServo;  // servo1
    private ServoImplEx ledServo;      // servo2 for LED control

    // ---- LED Color Enum ----
    public enum LedColor {
        RED(0.3),
        ORANGE(0.333),
        YELLOW(0.388),
        GREEN(0.50),
        BLUE(0.611),
        VIOLET(0.722),
        WHITE(1.0),
        OFF(0.0);

        public final double pwmValue;

        LedColor(double pwmValue) {
            this.pwmValue = pwmValue;
        }
    }
    /**
     * Initializes all servos and the PID controller for the turret.
     */
    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, "shservo1");
        ledServo = hardwareMap.get(ServoImplEx.class, "shservo0");
        ledServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Center the turret on initialization
        setTurretServoPos(0.5);

        // Initialize the PID controller with our constants
        pidController.initPID(TURRET_P, TURRET_I, TURRET_D);
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

    /**
     * Sets the color of the goBILDA RGB LED status indicator.
     * @param color The desired color from the LedColor enum.
     */
    public static void setLedColor(LedColor color) {
        if (ledServo == null || color == null) {
            return;
        }
        ledServo.setPosition(color.pwmValue);
    }

    public void setIndexerServoPos(double pos) {
        if (indexerServo != null) indexerServo.setPosition(pos);
    }

}
