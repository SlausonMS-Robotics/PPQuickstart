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
    private final other_helpers distanceAverage = new other_helpers();

    private final Timer targetTimer = new Timer();
    private final Timer llTimer = new Timer();



    private boolean target_acquired = false;
    private double llGoalDist = 0;
    private double llGoalHeadingError = 0;
    private double speed = 0;
    private int shooterSpeedAdjust = 0;
    private double turretPosAdjust = 0;
    private static final int ticks_per_rev = 28;

    public TurretAiming(Follower follower, limelight3A limelight, servos Servos, motors robotMotors, Telemetry telemetry) {
        this.follower = follower;
        this.limelight = limelight;
        this.Servos = Servos;
        this.robotMotors = robotMotors;
        this.telemetry = telemetry;

        llTimer.resetTimer();
        targetTimer.resetTimer();

        headingAverage.initMovingAverage(3);
        distanceAverage.initMovingAverage(3);
    }

    public void update(String myAllianceColor) {
        if (llTimer.getElapsedTime() >= 10) {
            LLResult result = limelight.limelight.getLatestResult();
            llTimer.resetTimer();

            if (result.isValid()) {
                target_acquired = true;
                servos.setLedColor(servos.LedColor.GREEN);
                targetTimer.resetTimer();

                llGoalHeadingError = result.getTx(); // took out averaging -> headingAverage.updateAndGetAverage(result.getTx());
                llGoalDist = result.getBotposeAvgDist(); // took out averaging -> distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());

            } else {
                servos.setLedColor(servos.LedColor.RED);
                if (targetTimer.getElapsedTime() >= 1000) {
                    target_acquired = false;
                    targetTimer.resetTimer();
                }
            }

            double goalDistance;
            double goalHeadingError;
            if (target_acquired) {
                goalDistance = llGoalDist;
                goalHeadingError = llGoalHeadingError;
            } else {
                goalDistance = other_helpers.distanceToGoal(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
                goalHeadingError = other_helpers.getHeadingErrorToGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), myAllianceColor);
            }

            // Set shooter velocity
            if (goalDistance > 0 && goalDistance < 5) {
                double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(goalDistance);
                speed = targetRPM * ticks_per_rev / 60;
                robotMotors.setShooterVelocity(speed + shooterSpeedAdjust);
            }

            // Adjust turret using PID
            if (Math.abs(goalHeadingError) > 0.1) {
                Servos.updateTurretWithPID(goalHeadingError + turretPosAdjust);
            }

            if (telemetry != null) {
                telemetry.addData("Target Acquired?", target_acquired);
                telemetry.addData("RPM", speed * 60 / ticks_per_rev);
                telemetry.addData("Goal Distance", goalDistance);
                telemetry.addData("Goal Heading Error", goalHeadingError);
                telemetry.addData("Current Pose", follower.getPose());
                telemetry.update();
            }
        }
    }

    public void setShooterSpeedAdjust(int adjustment) {
        this.shooterSpeedAdjust = adjustment;
    }

    public void setTurretPosAdjust(double adjustment) {
        this.turretPosAdjust = adjustment;
    }
}
