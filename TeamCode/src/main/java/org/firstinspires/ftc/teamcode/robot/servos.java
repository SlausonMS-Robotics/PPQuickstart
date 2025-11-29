package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class servos {

    // ---- Constants ----

    // The physical angle limits of the turret in degrees. Adjust these to match your hardware.
    public static final double TURRET_MIN_ANGLE_DEG = -60.0;
    public static final double TURRET_MAX_ANGLE_DEG = 60.0;

    // ---- Physical Conversion Constants ----
    private static final double GEAR_RATIO = 86.0 / 42.0; // Turret Gear / Servo Gear
    private static final double SERVO_DEGREES_RANGE = 1400.0; // Effective range of a 5-turn servo (5 * 280 deg)
    private static final double TURRET_CENTER_POS = 0.5; // The raw servo position that corresponds to a 0-degree turret angle.

    // SERVO_UNITS_PER_DEGREE: The scaling factor to convert degrees of turret rotation to servo units.
    private static final double SERVO_UNITS_PER_DEGREE = GEAR_RATIO / SERVO_DEGREES_RANGE;

    // ---- PID Constants ----
    public static final double TURRET_P = 0.1;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.0;

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

        setTurretAngleDeg(0.0);
        pidController.initPID(TURRET_P, TURRET_I, TURRET_D);
    }

    /**
     * Sets the color of the goBILDA RGB LED status indicator.
     * This is now an INSTANCE method.
     * @param color The desired color from the LedColor enum.
     */
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
     * Manually moves the turret by a given number of degrees.
     * @param angleIncrementDeg The number of degrees to move the turret.
     */
    public void moveTurretManually(double angleIncrementDeg) {
        double currentAngle = getTurretAngleDeg();
        double newAngle = currentAngle + angleIncrementDeg;
        setTurretAngleDeg(newAngle);
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
     * @param headingErrorDeg The error in degrees between the current and target heading.
     */
    public void updateTurretWithPID(double headingErrorDeg) {
        double pidCorrectionDeg = pidController.updatePID(headingErrorDeg, 0);
        double currentAngleDeg = getTurretAngleDeg();
        double newAngleDeg = currentAngleDeg + pidCorrectionDeg;
        setTurretAngleDeg(newAngleDeg);
    }

    /**
     * Sets the turret to a specific angle in degrees, respecting the physical limits.
     * @param angleDeg The desired angle for the turret.
     */
    public void setTurretAngleDeg(double angleDeg) {
        double clippedAngle = Range.clip(angleDeg, TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
        double pos = getServoPosFromAngle(clippedAngle);
        turretServo.setPosition(pos);
    }

    private double getServoPosFromAngle(double angleDeg) {
        return TURRET_CENTER_POS + (angleDeg * SERVO_UNITS_PER_DEGREE);
    }

    public double getTurretAngleDeg() {
        double pos = turretServo.getPosition();
        return (pos - TURRET_CENTER_POS) / SERVO_UNITS_PER_DEGREE;
    }

    public double getFieldCentricTurretHeading(double robotHeadingDeg) {
        double turretFieldHeading = robotHeadingDeg + getTurretAngleDeg();
        while (turretFieldHeading <= -180) {
            turretFieldHeading += 360;
        }
        while (turretFieldHeading > 180) {
            turretFieldHeading -= 360;
        }
        return turretFieldHeading;
    }
}
