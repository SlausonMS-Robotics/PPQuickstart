package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class servos {

    // ---- Constants ----



    public static final double TURRET_MAX_DEG = 45;
    public static final double TURRET_MIN_DEG = -45;

    // ---- Physical Conversion Constants ----
    private static final double GEAR_RATIO = 86.0 / 42.0; // Turret Gear / Servo Gear
    private static final double SERVO_DEGREES_RANGE = 355.0; // Effective range of a 5-turn servo (5 * 280 deg)
    private static final double TURRET_CENTER_POS = 0.5; // The raw servo position that corresponds to a 0-degree turret angle.

    // SERVO_UNITS_PER_DEGREE: The scaling factor to convert degrees of turret rotation to servo units.
    private static final double SERVO_UNITS_PER_DEGREE = (GEAR_RATIO / SERVO_DEGREES_RANGE);

    // ---- PID Constants ----
    public static final double TURRET_P = .7;
    public static final double TURRET_I = 0.0;
    public static final double TURRET_D = 0.00001;

    // ---- PID Controller ----
    private other_helpers pidController = new other_helpers();

    // ---- Servos ----
    private ServoImplEx turretServo, ledServo, intakeServo1, intakeServo2, bouncerServo;
    
    // ---- Telemetry ----
    private Telemetry telemetry;
    static double BOUNCE_POS_1 = .33; //Far
    static double MID_BOUNCE_POS = .42;
    static double CLOSE_BOUNCE_POS = .52;

    public enum LedColor { GREEN, RED, VIOLET, YELLOW, OFF }

    private LedColor LedState;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        turretServo = hardwareMap.get(ServoImplEx.class, "shservo0");
        ledServo = hardwareMap.get(ServoImplEx.class, "shservo1");
        intakeServo1 = hardwareMap.get(ServoImplEx.class, "shservo4"); //left
        intakeServo2 = hardwareMap.get(ServoImplEx.class, "shservo5"); //right
        bouncerServo = hardwareMap.get(ServoImplEx.class, "shservo3");


        ledServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        turretServo.setPosition(.5);
        pidController.initPID(TURRET_P, TURRET_I, TURRET_D);
    }

    public void setLedColor(LedColor color) {
        switch (color) {
            case GREEN:
                if(LedState != color) ledServo.setPosition(.5);
                break;
            case RED:
                if(LedState != color) ledServo.setPosition(0.28);
                break;
            case VIOLET:
                if(LedState != color) ledServo.setPosition(0.72);
                break;
            case YELLOW:
                if(LedState != color) ledServo.setPosition(0.39);
                break;
            case OFF:
            default:
                if(LedState != color) ledServo.setPosition(0.0);
                color = LedColor.OFF;
                break;
        }
        LedState = color;

    }

    public void setBouncerServo(double pos) {
        bouncerServo.setPosition(pos);

    }

    public double getBouncerServoPos(){
        return bouncerServo.getPosition();
    }

    /**
     * Updates the turret position using a PID controller to point towards a target field heading.
     * @param targetFieldHeadingDeg The desired field-centric heading for the turret.
     * @param robotHeadingDeg The robot's current field-centric heading.
     */
    public void updateTurretWithPID(double targetFieldHeadingDeg, double robotHeadingDeg) {







        // Get the PID correction in degrees
        double pidCorrectionDeg = -pidController.updatePID(robotHeadingDeg, targetFieldHeadingDeg);
        

        incrementTurretInDegrees(pidCorrectionDeg);

        if (telemetry != null) {
            //telemetry.addData("Target Field Heading", "%.2f", targetFieldHeadingDeg);
            telemetry.addData("Turret Pos", "%.2f", turretServo.getPosition());
            //telemetry.addData("Robot Current Heading", "%.2f", robotHeadingDeg);
            //telemetry.addData("Heading Error", "%.2f", headingError);
            telemetry.addData("Heading Error", "%.2f", robotHeadingDeg);
            telemetry.addData("PID Correction (Deg)", "%.2f", pidCorrectionDeg);
            telemetry.addData("PID Correction (pos)", "%.2f", pidCorrectionDeg * SERVO_UNITS_PER_DEGREE);
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

    public void incrementTurretInDegrees(double degInc){
        if(Math.abs(degInc) >= .001) {
            double curPos = turretServo.getPosition();
            double degIncrementToPos = degInc * SERVO_UNITS_PER_DEGREE;
            double newPos = curPos + degIncrementToPos;
            //if (newPos < TURRET_CENTER_POS + (TURRET_MIN_DEG * SERVO_UNITS_PER_DEGREE)) newPos = TURRET_CENTER_POS + ((TURRET_MAX_DEG - 5) * SERVO_UNITS_PER_DEGREE);
            //else if (newPos > TURRET_CENTER_POS + (TURRET_MAX_DEG * SERVO_UNITS_PER_DEGREE)) newPos = TURRET_CENTER_POS + ((TURRET_MIN_DEG + 5) * SERVO_UNITS_PER_DEGREE);
            //if(newPos > TURRET_MAX_DEG * SERVO_UNITS_PER_DEGREE) newPos = TURRET_MAX_DEG * SERVO_UNITS_PER_DEGREE;
            //else if(newPos < TURRET_MIN_DEG * SERVO_UNITS_PER_DEGREE) newPos = TURRET_MIN_DEG * SERVO_UNITS_PER_DEGREE;
            turretServo.setPosition(newPos);
        }

    }

    /**
     * Sets the turret to a specific robot-centric angle, respecting the physical limits.
     * @param angleDeg The desired robot-centric angle for the turret.
     */


    /**
     * Gets the turret's current angle relative to the robot in degrees.
     * @return The turret's robot-centric angle in degrees.
     */
    public double getTurretRobotAngleDeg() {
        double pos = turretServo.getPosition(); //.5 = 0 degrees, positive is to the right
        return (pos - TURRET_CENTER_POS) / SERVO_UNITS_PER_DEGREE; //center position is facing forward
    }

    public void setTurretServoPos(double pos) {
        turretServo.setPosition(pos);
    }



    public double getTurretFieldAngleDeg(double robotHeadingDeg) {
        return robotHeadingDeg - getTurretRobotAngleDeg();
    }



}
