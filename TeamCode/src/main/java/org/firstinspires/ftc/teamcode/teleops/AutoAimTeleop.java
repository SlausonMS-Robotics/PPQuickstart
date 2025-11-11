package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.robot.other_helpers.blueGoalX;
import static org.firstinspires.ftc.teamcode.robot.other_helpers.blueGoalY;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.localizer_fusion;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.other_helpers;
import org.firstinspires.ftc.teamcode.robot.servos;

/**
 * This is an example TeleOp for testing out the 2360 robot. It'll need to be updated to allow for
 * targeting other AprilTags
 */
@TeleOp(name = "Auto Aim Teleop", group = "23609")
public class AutoAimTeleop extends OpMode {
    private static final int MOVING_AVERAGE_SIZE = 3;
    private static double scalar = 1.0;
    private static final int ticks_per_rev = 28;

    private double ll_goal_dist = 0;
    private double ll_goal_heading = 0;
    private Timer targetTimer, llTimer, poseTimer, llPoseTimer;
    motors shooter = new motors();
    limelight3A limelight = new limelight3A();
    servos Servos = new servos(); // Use the servos class
    private localizer_fusion fusion;

    // Moving Average Helpers
    private other_helpers headingAverage = new other_helpers();
    private other_helpers distanceAverage = new other_helpers();

    Follower follower;
    private boolean use_PP = true; // Enabled Pedro Pathing for fusion
    private final Pose startPose = new Pose(0,0,0);

    double curBotPoseX = 0;
    double curBotPoseY = 0;
    double curBotPoseHead = 0;
    private boolean target_acquired = false;

    private int scanCW = 1;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        // Initialize all our robot hardware
        shooter.init(hardwareMap);
        Servos.init(hardwareMap); // This now initializes the turret servo and PID
        limelight.init(hardwareMap,0, telemetry);

        if (use_PP) {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);
            fusion = new localizer_fusion(follower, limelight);
            getCurBotPose();
        }

        headingAverage.initMovingAverage(MOVING_AVERAGE_SIZE);
        distanceAverage.initMovingAverage(MOVING_AVERAGE_SIZE);

        targetTimer = new Timer();
        llTimer = new Timer();
        poseTimer = new Timer();
        llPoseTimer = new Timer();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        if(use_PP) {
            follower.startTeleopDrive(false);
            poseTimer.resetTimer();
            llPoseTimer.resetTimer();
        }
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        long now = System.nanoTime();

        if(use_PP) {
            // Your existing drive code...
            if (gamepad1.left_trigger > .1) {
                scalar = .5;
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 1), Math.pow(-gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar, 1), false);
            } else {
                scalar = 1.0;
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 3), Math.pow(-gamepad1.right_stick_x * scalar, 3), false);
            }

            follower.update();
            fusion.pushOdomSample(now); // Push new odometry data for fusion

            if (poseTimer.getElapsedTime() >= 10) { //update servo tracking based on current pose every x ms
                getCurBotPose();
                Servos.pointTurretToGoal(curBotPoseX, curBotPoseY, curBotPoseHead, blueGoalX, blueGoalY);
                poseTimer.resetTimer();
            }
            if (llPoseTimer.getElapsedTime() >= 100) { // Fused pose update from Limelight
                fusion.updatePoseFromLimelight(now);
                llPoseTimer.resetTimer();
            }
        }

        if(llTimer.getElapsedTime() >= 100) {
            LLResult result = limelight.limelight.getLatestResult();
            llTimer.resetTimer();

            if (result.isValid()) {
                target_acquired = true;
                targetTimer.resetTimer();

                ll_goal_heading = headingAverage.updateAndGetAverage(result.getTx());
                ll_goal_dist = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());

                telemetry.addData("dist", ll_goal_dist);
                telemetry.addData("heading", ll_goal_heading);
            } else {
                if (targetTimer.getElapsedTime()>=2000){
                    target_acquired = false;
                    targetTimer.resetTimer();
                }
            }

            if(target_acquired) {
                // Set shooter velocity
                if (ll_goal_dist > 0.1 && ll_goal_dist < 4) {
                    double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(ll_goal_dist);
                    double speed = targetRPM * ticks_per_rev / 60;
                    shooter.setShooterVelocity(speed);
                    telemetry.addData("RPM", speed * 60 / ticks_per_rev);
                }

                // Adjust turret using PID
                if (Math.abs(ll_goal_heading) > 0.01) {
                    // The heading from limelight is in degrees. Convert to radians for the PID controller.
                    double headingErrorRadians = Math.toRadians(ll_goal_heading);

                    // Call the new method to update the turret position
                    Servos.updateTurretWithPID(headingErrorRadians);

                    // Update scan direction for when we lose the target
                    if(headingErrorRadians >= 0){
                        scanCW = -1; // If error is positive (target to the right), next scan should be left
                    } else {
                        scanCW = 1; // If error is negative (target to the left), next scan should be right
                    }

                    telemetry.addData("position", Servos.getTurretServoPos());
                    telemetry.addData("heading_error_rad", headingErrorRadians);
                }
                telemetry.addData("Target Acquired", target_acquired);
            }
            else {
                // Scan for the AprilTag
                double newScanPos = Servos.getTurretServoPos() + (0.0007 * scanCW);
                Servos.setTurretServoPos(newScanPos);
                if (Servos.getTurretServoPos() >= servos.TURRET_MAX_POS || Servos.getTurretServoPos() <= servos.TURRET_MIN_POS){
                    scanCW *= -1;
                }
                telemetry.addData("Target Acquired", target_acquired);
            }
            telemetry.update();
        }
    }

    public void getCurBotPose() {
        if (use_PP) {
            curBotPoseX = follower.getPose().getX();
            curBotPoseY = follower.getPose().getY();
            curBotPoseHead = follower.getPose().getHeading();
        }
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
