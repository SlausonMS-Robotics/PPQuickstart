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
        double odomDistance = other_helpers.distanceToGoal(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        if (odomDistance > 0 && odomDistance < 5) {
            double targetRPM = other_helpers.getRPMForDistance(odomDistance);
            double speed = targetRPM * ticks_per_rev / 60;
            robotMotors.setShooterVelocity(speed);
        }

        // Aim with odometry
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        double odomHeadingErrorDeg = other_helpers.getHeadingErrorToGoal(follower.getPose().getX(), follower.getPose().getY(), robotHeadingDeg, myAllianceColor);
        
        if (Math.abs(odomHeadingErrorDeg) > 0.1) {
            Servos.updateTurretWithPID(odomHeadingErrorDeg);
        }
        
        if (telemetry != null) {
            telemetry.addData("AIMING MODE", "ODOMETRY (DEFAULT)");
            telemetry.addData("Odom Heading Error", odomHeadingErrorDeg);
        }
        
        updateTelemetry();
    }
    
    /**
     * Explicitly polls the Limelight and attempts to aim.
     * Call this when you want to override odometry aiming with vision.
     * @return true if a valid target was found and used for aiming, false otherwise.
     */
    public boolean updateLimelightAiming() {
        if (llTimer.getElapsedTime() < 10) {
            return false; // Throttle the polling
        }
        llTimer.resetTimer();

        // Defensively check if the limelight object is null before using it.
        LLResult result = null;
        if (limelight != null) {
            result = limelight.limelight.getLatestResult();
        }

        if (result != null && result.isValid()) {
            // --- LIMELIGHT AIMING ---
            Servos.setLedColor(servos.LedColor.GREEN);

            double llGoalHeadingError = result.getTx();
            double llGoalDist = result.getBotposeAvgDist();

            // Set shooter velocity based on Limelight distance
            if (llGoalDist > 0 && llGoalDist < 5) {
                double targetRPM = other_helpers.getRPMForDistance(llGoalDist);
                double speed = targetRPM * ticks_per_rev / 60;
                robotMotors.setShooterVelocity(speed);
            }

            // Aim turret using Limelight heading
            if (Math.abs(llGoalHeadingError) > 0.1) {
                Servos.updateTurretWithPID(llGoalHeadingError);
            }

            if (telemetry != null) {
                telemetry.addData("AIMING MODE", "LIMELIGHT");
                telemetry.addData("LL Heading Error", llGoalHeadingError);
                telemetry.addData("LL Distance", llGoalDist);
            }
            updateTelemetry();
            return true; // Success
        }
        
        // No valid target, so return false
        return false;
    }
    
    /**
     * Private helper to update common telemetry data.
     */
    private void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.update();
        }
    }
}
