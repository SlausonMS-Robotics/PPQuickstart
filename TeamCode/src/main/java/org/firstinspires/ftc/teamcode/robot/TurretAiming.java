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
    private final other_helpers headingAverage = new other_helpers();
    private final other_helpers helpers = new other_helpers();
    private final other_helpers distanceAverage = new other_helpers();

    private final Timer targetTimer = new Timer();
    private final Timer llTimer = new Timer();

    private boolean targetAcquired = false;

    public TurretAiming(Follower follower, limelight3A limelight, servos Servos, motors robotMotors, Telemetry telemetry) {
        this.follower = follower;
        this.limelight = limelight;
        this.Servos = Servos;
        this.robotMotors = robotMotors;
        this.telemetry = telemetry;

        headingAverage.initMovingAverage(2);
        distanceAverage.initMovingAverage(2);
    }

    /**
     * Main update loop for turret aiming.
     * @param myAllianceColor The current alliance color ("blue" or "red").
     * @param useOdometry If true, aims using only odometry. If false, uses Limelight with odometry fallback.
     */
    public void update(String myAllianceColor, boolean useOdometry) {
        double goalDistanceMeters;
        double goalHeadingErrorDeg;
        goalDistanceMeters = other_helpers.distanceToGoal(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        goalHeadingErrorDeg = other_helpers.getHeadingErrorToGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), myAllianceColor);

        if (!useOdometry) {

            // --- VISION AIMING WITH ODOMETRY FALLBACK ---
            if (llTimer.getElapsedTime() >= 10) {
                LLResult result = limelight.limelight.getLatestResult();
                llTimer.resetTimer();

                if (result.isValid()) {
                    targetAcquired = true;
                    Servos.setLedColor(servos.LedColor.GREEN);
                    targetTimer.resetTimer();

                    // Use vision data for distance and heading
                    goalDistanceMeters = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());
                    goalHeadingErrorDeg = headingAverage.updateAndGetAverage(result.getTx());

                } else {


                    if (targetTimer.getElapsedTime() >= 500) {
                        targetAcquired = false;
                        Servos.setLedColor(servos.LedColor.RED);
                    }
                }
            }
        }


        // --- SHARED AIMING LOGIC ---
        double targetRPM = helpers.getRPMForDistance(goalDistanceMeters);
        // Set shooter velocity
        if (goalDistanceMeters > 0.1 && goalDistanceMeters < 5) {

            robotMotors.setShooterVelocity(targetRPM);
        }

        // Adjust turret using PID
        if (Math.abs(goalHeadingErrorDeg) > .25) { // Deadband in degrees
            Servos.updateTurretWithPID(goalHeadingErrorDeg);
        }

        if (telemetry != null) {
            telemetry.addData("Aiming Mode", (useOdometry ? "Odometry" : (targetAcquired ? "Vision" : "Odom Fallback")));
            telemetry.addData("RPM", targetRPM);
            telemetry.addData("Goal Distance", goalDistanceMeters);
            telemetry.addData("Goal Heading Error", Math.toDegrees(goalHeadingErrorDeg));
            telemetry.update();
        }
    }

}
