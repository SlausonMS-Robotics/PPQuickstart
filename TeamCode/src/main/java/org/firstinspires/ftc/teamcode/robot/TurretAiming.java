package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretAiming {

    private final Follower follower;
    private final limelight3A limelight;
    private final servos Servos;
    private final motors robotMotors;
    private final Telemetry telemetry;

    private final Timer llTimer = new Timer();
    private static final int ticks_per_rev = 28;

    // ---- Aiming Constants (moved from other_helpers) ----
    private static final int blueGoalX = 12;
    private static final int blueGoalY = 136;
    private static final int redGoalX = 132;
    private static final int redGoalY = 136;

    // ---- Polynomial RPM Coefficients ----
    // These coefficients define the quadratic equation: RPM = A*x^2 + B*x + C
    // where x is the distance in meters. Derived from the new stepped data.
    private static final double POLY_A = 200.0;
    private static final double POLY_B = -100.0;
    private static final double POLY_C = 2700.0;

    public TurretAiming(Follower follower, limelight3A limelight, servos Servos, motors robotMotors, Telemetry telemetry) {
        this.follower = follower;
        this.limelight = limelight;
        this.Servos = Servos;
        this.robotMotors = robotMotors;
        this.telemetry = telemetry;

        llTimer.resetTimer();
    }

    /**
     * Aims the turret using odometry data as the default.
     * This should be called in the main loop for default behavior.
     */
    public void updateOdomAiming(String myAllianceColor) {
        // This is now the default aiming method.
         // Default to red (no vision)

        // Set shooter speed based on odometry
        double odomDistanceMeters = .0254 * distanceToGoalInches(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        if (odomDistanceMeters > .05 && odomDistanceMeters < 5) {
            double targetRPM = getRPMForDistancePolyMeters(odomDistanceMeters);
            double speed = targetRPM * ticks_per_rev / 60;
            robotMotors.setShooterVelocity(speed);
        }

        // Aim with odometry
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        // Directly get the absolute field heading of the goal.
        double targetFieldHeadingDeg = getFieldHeadingToGoal(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        
        // Let the PID controller handle aiming. It will do nothing if the error is tiny.
        Servos.updateTurretWithPID(targetFieldHeadingDeg, robotHeadingDeg);
        
        if (telemetry != null) {
           // telemetry.addData("AIMING MODE", "ODOMETRY (DEFAULT)");
            //telemetry.addData("Target Field Heading", "%.2f", targetFieldHeadingDeg);
            //telemetry.addData("Current Field Heading", "%.2f", robotHeadingDeg);
            }
        
        updateTelemetry();
    }
    
    /**
     * Explicitly polls the Limelight and attempts to aim.
     * @return true if a valid target was found and used for aiming, false otherwise.
     */

    private void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.update();
        }
    }

    /**
     * Calculates flywheel RPM using a quadratic polynomial for smooth, continuous speed scaling.
     * @param rangeMeters The distance to the target in meters.
     * @return The calculated RPM for the flywheel.
     */
    public static double getRPMForDistancePolyMeters(double rangeMeters) {
        if (rangeMeters <= 1) return POLY_A + POLY_B + POLY_C;
        return POLY_A * Math.pow(rangeMeters, 2) + POLY_B * rangeMeters + POLY_C;
    }

    public static double getRPMForDistanceStepMeters(double rangeMeters) {
        if (rangeMeters <= 1.0) return 2800; // Base close-range shot
        if (rangeMeters <= 1.5) return 3000; // +200
        if (rangeMeters <= 2.0) return 3300; // +300
        if (rangeMeters <= 2.5) return 3700; // +400
        if (rangeMeters <= 3.0) return 4200; // +500
        return 4500; // Max power shot for anything over 3m
    }
   

    private double distanceToBlueGoal(double currentX, double currentY) {
        double deltaX = blueGoalX - currentX;
        double deltaY = blueGoalY - currentY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    private double distanceToRedGoal(double currentX, double currentY) {
        double deltaX = redGoalX - currentX;
        double deltaY = redGoalY - currentY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    // These methods now return the absolute field heading to the goal.
    private double getFieldHeadingToBlueGoal(double currentX, double currentY) {
        return Math.toDegrees(Math.atan2(blueGoalY - currentY, blueGoalX - currentX));
    }

    private double getFieldHeadingToRedGoal(double currentX, double currentY) {
        return Math.toDegrees(Math.atan2(redGoalY - currentY, redGoalX - currentX));
    }
    
    private double distanceToGoalInches(double currentX, double currentY, String allianceColor) {
        if ("blue".equals(allianceColor)) {
            return distanceToBlueGoal(currentX, currentY);
        } else {
            return distanceToRedGoal(currentX, currentY);
        }
    }

    private double getFieldHeadingToGoal(double currentX, double currentY, String allianceColor) {
        if ("blue".equals(allianceColor)) {
            return getFieldHeadingToBlueGoal(currentX, currentY);
        } else {
            return getFieldHeadingToRedGoal(currentX, currentY);
        }
    }
}
