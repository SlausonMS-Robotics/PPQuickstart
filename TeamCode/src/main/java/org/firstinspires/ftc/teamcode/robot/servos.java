package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class servos {

    // ---- Constants ----

    // The physical angle limits of the turret in degrees. Adjust these to match your hardware.
    public static final double TURRET_MIN_ANGLE_DEG = -30.0;
    public static final double TURRET_MAX_ANGLE_DEG = 210.0;

    // ---- Physical Conversion Constants ----
    private static final double GEAR_RATIO = 86.0 / 42.0; // Turret Gear / Servo Gear
    private static final double SERVO_DEGREES_RANGE = 1400.0; // Effective range of a 5-turn servo (5 * 280 deg)
    private static final double TURRET_CENTER_POS = 0.5; // The raw servo position that corresponds to a 0-degree turret angle.

    // SERVO_UNITS_PER_DEGREE: The scaling factor to convert degrees of turret rotation to servo units.
    private static final double SERVO_UNITS_PER_DEGREE = (GEAR_RATIO / SERVO_DEGREES_RANGE);

    // ---- PID Constants ----
    public static final double TURRET_P = 0.05;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.005;

    // ---- PID Controller ----
    private other_helpers pidController = new other_helpers();

    // ---- Servos ----
    private ServoImplEx turretServo, ledServo, intakeServo1, intakeServo2;
    
    // ---- Telemetry ----
    private Telemetry telemetry;


    public enum LedColor { GREEN, RED, VIOLET, OFF }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretServo = hardwareMap.get(ServoImplEx.class, "shservo0");
        ledServo = hardwareMap.get(ServoImplEx.class, "shservo1");
        intakeServo1 = hardwareMap.get(ServoImplEx.class, "shservo3");
        intakeServo2 = hardwareMap.get(ServoImplEx.class, "shservo4");

        ledServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        setTurretRobotAngle(0.0);
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
     * Updates the turret position using a PID controller to point towards a target field heading.
     * @param targetFieldHeadingDeg The desired field-centric heading for the turret.
     * @param robotHeadingDeg The robot's current field-centric heading.
     */
    public void updateTurretWithPID(double targetFieldHeadingDeg, double robotHeadingDeg) {


        // 2. Calculate the error in field coordinates
        double headingError = targetFieldHeadingDeg - getTurretFieldAngle(robotHeadingDeg);
        ;

        // 3. Get the PID correction in degrees
        double pidCorrectionDeg = pidController.updatePID(headingError, 0);
        
        // 6. Command the turret to the new robot-centric angle
        setTurretRobotAngle(robotHeadingDeg + pidCorrectionDeg);

        if (telemetry != null) {
            telemetry.addData("Target Field Heading", "%.2f", targetFieldHeadingDeg);
            telemetry.addData("Robot Current Heading", "%.2f", robotHeadingDeg);
            telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("PID Correction (Deg)", "%.2f", pidCorrectionDeg);
        }
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
     * Sets the turret to a specific robot-centric angle, respecting the physical limits.
     * @param angleDeg The desired robot-centric angle for the turret.
     */
    public void setTurretRobotAngle(double angleDeg) {
        double clippedAngle = Range.clip(angleDeg, TURRET_MIN_ANGLE_DEG, TURRET_MAX_ANGLE_DEG);
        double pos = getServoPosFromAngle(clippedAngle);
        turretServo.setPosition(pos);
    }

    /**
     * Gets the turret's current angle relative to the robot in degrees.
     * @return The turret's robot-centric angle in degrees.
     */
    public double getTurretRobotAngle() {
        double pos = turretServo.getPosition();
        return (pos - TURRET_CENTER_POS) / SERVO_UNITS_PER_DEGREE;
    }

    private double getServoPosFromAngle(double angleDeg) {
        return TURRET_CENTER_POS + (angleDeg * SERVO_UNITS_PER_DEGREE);
    }

    public double getFieldCentricTurretHeading(double robotHeadingDeg) {
        // The field heading is the angle of the robot's chassis (its "front")
        // plus the angle of the turret relative to the chassis.
        // But since the turret angle is measured from the robot's RIGHT side, we must first find the angle of the right side.
        double robotRightSideAngle = robotHeadingDeg - 90;
        double turretFieldHeading = robotRightSideAngle + getTurretRobotAngle();
        return getNormalizedError(turretFieldHeading);
    }

    public double getTurretFieldAngle(double robotHeadingDeg) {
        return robotHeadingDeg + getTurretRobotAngle();
    }


    private double getNormalizedError(double angle) {
        while (angle <= -180) angle += 360;
        while (angle > 180) angle -= 360;
        return angle;
    }
}
