package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;

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
        Servos.setLedColor(servos.LedColor.RED); // Default to red (no vision)

        // Set shooter speed based on odometry
        double odomDistance = distanceToGoalInches(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        if (odomDistance > 0 && odomDistance < 5) {
            double targetRPM = FlywheelShooter.getRPMForDistance(odomDistance);
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
            telemetry.addData("AIMING MODE", "ODOMETRY (DEFAULT)");
            telemetry.addData("Target Field Heading", "%.2f", targetFieldHeadingDeg);
            telemetry.addData("Current Field Heading", "%.2f", robotHeadingDeg);
            }
        
        updateTelemetry();
    }
    
    /**
     * Explicitly polls the Limelight and attempts to aim.
     * @return true if a valid target was found and used for aiming, false otherwise.
     */
    public boolean updateLimelightAiming() {
        if (llTimer.getElapsedTime() < 10) {
            return false; // Throttle the polling
        }
        llTimer.resetTimer();

        LLResult result = null;
        if (limelight != null) {
            result = limelight.limelight.getLatestResult();
        }

        if (result != null && result.isValid()) {
            Servos.setLedColor(servos.LedColor.GREEN);

            double llGoalHeadingError = result.getTx();
            double llGoalDist = result.getBotposeAvgDist();

            if (llGoalDist > 0 && llGoalDist < 5) {
                double targetRPM = FlywheelShooter.getRPMForDistance(llGoalDist);
                double speed = targetRPM * ticks_per_rev / 60;
                robotMotors.setShooterVelocity(speed);
            }

            double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
            double currentTurretFieldHeading = Servos.getFieldCentricTurretHeading(robotHeadingDeg);
            double targetFieldHeading = currentTurretFieldHeading - llGoalHeadingError;

            // Let the PID controller handle aiming. It will do nothing if the error is tiny.
            Servos.updateTurretWithPID(targetFieldHeading, robotHeadingDeg);

            if (telemetry != null) {
                telemetry.addData("AIMING MODE", "LIMELIGHT");
                telemetry.addData("Target Field Heading", "%.2f", targetFieldHeading);
                telemetry.addData("LL Distance", llGoalDist);
            }
            updateTelemetry();
            return true; // Success
        }
        
        return false;
    }
    
    private void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.update();
        }
    }

    // ---- Aiming Logic Moved from other_helpers ----

    private static class FlywheelShooter {
        private static final double RPM_PER_METER = 200;
        private static final double MAX_RPM = 4200;
        private static final double BASE_RPM = 3400;

        public static double getRPMForDistance(double rangeMeters) {
            if (rangeMeters <= 2) return BASE_RPM;
            if (rangeMeters >= 3) return MAX_RPM;
            return BASE_RPM + (rangeMeters * RPM_PER_METER);
        }
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
