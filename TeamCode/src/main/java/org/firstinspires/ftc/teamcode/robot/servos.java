package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class servos {

    // ---- Constants ----

    public static final double TURRET_MIN_POS = .3;
    public static final double TURRET_MAX_POS = .7;
    // The physical angle limits of the turret in radians. Adjust these to match your hardware.
    public static final double TURRET_MIN_ANGLE_RAD = -Math.PI / 2.0; // -90 degrees
    public static final double TURRET_MAX_ANGLE_RAD = Math.PI / 2.0; // +90 degrees


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

    /**
     * Initializes all servos and the PID controller for the turret.
     */
    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(ServoImplEx.class, "servo0");
        indexerServo = hardwareMap.get(ServoImplEx.class, "servo1");

        // Center the turret on initialization
        setTurretServoPos(0.5);

        // Initialize the PID controller with our constants
        pidController.initPID(TURRET_P, TURRET_I, TURRET_D);
    }

    /**
     * Points the turret toward a given set of field coordinates using a PID controller.
     * @param robotX The robot's current X position.
     * @param robotY The robot's current Y position.
     * @param robotHeading The robot's current heading in radians.
     * @param goalX The target's X coordinate.
     * @param goalY The target's Y coordinate.
     */
    public void pointTurretToGoal(double robotX, double robotY, double robotHeading, double goalX, double goalY) {
        // Step 1: Calculate the world angle from the robot to the goal.
        double worldAngleToGoal = Math.atan2(goalY - robotY, goalX - robotX);

        // Step 2: Calculate the desired turret angle relative to the robot's current heading.
        double targetTurretAngle = worldAngleToGoal - robotHeading;

        // Step 3: Normalize the target angle to the range [-PI, PI] to find the shortest path.
        while (targetTurretAngle > Math.PI) {
            targetTurretAngle -= 2 * Math.PI;
        }
        while (targetTurretAngle <= -Math.PI) {
            targetTurretAngle += 2 * Math.PI;
        }

        // Step 4: Get the turret's current angle.
        double currentTurretAngle = getTurretAngle();

        // Step 5: Calculate the error that the PID controller needs to correct.
        double turretError = targetTurretAngle - currentTurretAngle;

        // Step 6: Normalize the error as well, as wrapping around might be the shortest path.
        while (turretError > Math.PI) {
            turretError -= 2 * Math.PI;
        }
        while (turretError <= -Math.PI) {
            turretError += 2 * Math.PI;
        }

        // Step 7: Pass the error to the PID updater.
        updateTurretWithPID(turretError);
    }


    /**
     * Updates the turret position using a PID controller to minimize heading error.
     * @param headingError The error in radians between the current and target heading.
     */
    public void updateTurretWithPID(double headingError) {
        // The PID controller calculates the necessary correction.
        // The `currentState` is our heading error, and the `targetState` is 0 (no error).
        double pidCorrection = pidController.updatePID(headingError, 0);

        // Get the current servo position
        double currentPos = getTurretServoPos();

        // Apply the correction to the current position.
        // A positive error means the target is at a larger angle than current,
        // so we should increase the servo position.
        double newPos = currentPos + pidCorrection;

        // Set the new position, letting the existing clipping handle the limits.
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
     * Gets the turret's current angle in radians, based on its servo position.
     * @return The turret's angle in radians.
     */
    private double getTurretAngle() {
        double pos = getTurretServoPos();
        // Linearly interpolate from servo position [MIN_POS, MAX_POS] to angle [MIN_ANGLE, MAX_ANGLE]
        double servoRange = TURRET_MAX_POS - TURRET_MIN_POS;
        if (servoRange <= 0) return 0; // Avoid division by zero
        double angleRange = TURRET_MAX_ANGLE_RAD - TURRET_MIN_ANGLE_RAD;
        return TURRET_MIN_ANGLE_RAD + ((pos - TURRET_MIN_POS) * angleRange / servoRange);
    }

    public void setIndexerServoPos(double pos) {
        if (indexerServo != null) indexerServo.setPosition(pos);
    }

}
