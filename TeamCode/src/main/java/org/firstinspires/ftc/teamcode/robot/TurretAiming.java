package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretAiming {

    private final Follower follower;
    private final limelight3A limelight;
    private final servos Servos;
    private final motors robotMotors;
    private final Telemetry telemetry;

    private final other_helpers helpers = new other_helpers();

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
    private static double FAR_LAUNCH_DIST= 3.2;
    private static final int ticks_per_rev = 28;
    private boolean isManualTurret = false;

    public TurretAiming(Follower follower, limelight3A limelight, servos Servos, motors robotMotors, Telemetry telemetry) {
        this.follower = follower;
        this.limelight = limelight;
        this.Servos = Servos;
        this.robotMotors = robotMotors;
        this.telemetry = telemetry;

        headingAverage.initMovingAverage(3);
        distanceAverage.initMovingAverage(3);
        //llGoalDist = FAR_LAUNCH_DIST;
    }

    public boolean isManualTurret() {
        return this.isManualTurret;
    }

    public void setManualTurret(boolean isManualTurret) {
        this.isManualTurret = isManualTurret;
    }



    public void update(String myAllianceColor) {
        if (llTimer.getElapsedTime() >= 10) {
            LLResult result = limelight.limelight.getLatestResult();
            llTimer.resetTimer();

            if (result.isValid()) {
                target_acquired = true;
                Servos.setLedColor(servos.LedColor.GREEN);
                targetTimer.resetTimer();

                llGoalHeadingError = headingAverage.updateAndGetAverage(result.getTx());
                llGoalDist = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());

            } else {
                Servos.setLedColor(servos.LedColor.RED);
                if (targetTimer.getElapsedTime() >= 500) {
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
                goalDistance = llGoalDist;
                goalHeadingError = llGoalHeadingError;
                //goalDistance = other_helpers.distanceToGoal(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
                //goalHeadingError = other_helpers.getHeadingErrorToGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), myAllianceColor);
            }

            // Set shooter velocity
            if (goalDistance > .1 && goalDistance < 5) {
                double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(goalDistance);
                speed = Range.clip((targetRPM + shooterSpeedAdjust) * ticks_per_rev / 60, helpers.MIN_RPM, helpers.MAX_RPM);
                robotMotors.setShooterVelocity(speed);
            }

            // Adjust turret using PID
            if (Math.abs(goalHeadingError) > 0.0) {
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
