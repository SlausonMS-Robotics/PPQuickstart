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
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
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

    private static final double intakePow = 1.0;
    private static final double tranferPow = 1.0;

    private boolean isIntake = false;
    private boolean isTransfer = false;
    private String myAllianceColor = "blue";
    private static final int MOVING_AVERAGE_SIZE = 3;
    private static double scalar = 1.0;
    private static final int ticks_per_rev = 28;

    private double ll_goal_dist = 0;
    private double ll_goal_heading = 0;
    private Timer targetTimer, llTimer, poseTimer, llPoseTimer, buttonDebounceTimer;
    motors robotMotors = new motors(); // Single object for all motors
    limelight3A limelight = new limelight3A();
    servos Servos = new servos(); // Use the servos class
    private localizer_fusion fusion;

    // Moving Average Helpers
    private other_helpers headingAverage = new other_helpers();
    private other_helpers distanceAverage = new other_helpers();


    Follower follower;
    private boolean use_PP = true; // Enabled Pedro Pathing for fusion
    private boolean use_LL = true;
    private boolean useFusion = false;
    private Pose startPose = new Pose(0,0,0);

    double curBotPoseX = 0;
    double curBotPoseY = 0;
    double curBotPoseHead = 0;
    private int shooterSpeedAjustIncrement = 50;
    private int shooterSpeedAdjust = 0;
    private double turretPosAdjustIncrement = .008;
    private double turretPosAdjust = 0;
    private boolean target_acquired = false;

    private int scanCW = 1;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        // Initialize all our robot hardware
        robotMotors.init(hardwareMap); //initializes all motors
        Servos.init(hardwareMap); // This now initializes the turret servo and PID
        limelight.init(hardwareMap,0, telemetry);

        if (use_PP) {
            follower = Constants.createFollower(hardwareMap);
            startPose = PoseStorage.currentPose;
            follower.setStartingPose(startPose);
            telemetry.addData("Starting Pose", startPose);
            telemetry.update();
            fusion = new localizer_fusion(follower, limelight);
            getCurBotPose();
        }

        headingAverage.initMovingAverage(MOVING_AVERAGE_SIZE);
        distanceAverage.initMovingAverage(MOVING_AVERAGE_SIZE);

        targetTimer = new Timer();
        llTimer = new Timer();
        poseTimer = new Timer();
        llPoseTimer = new Timer();
        buttonDebounceTimer = new Timer();
        
        myAllianceColor = PoseStorage.allianceColor;
        telemetry.addData("Alliance Color", myAllianceColor);
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {

        if(buttonDebounceTimer.getElapsedTime() > 500 && (gamepad1.start || gamepad2.start)){
            buttonDebounceTimer.resetTimer();
            if("blue".equals(myAllianceColor)){
                myAllianceColor = "red";
            }
            else {
                myAllianceColor = "blue";
            }
            telemetry.addData("Alliance Color", myAllianceColor);
            telemetry.update();
            
        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        if(use_PP) {
            follower.startTeleopDrive(true);
            poseTimer.resetTimer();
            llPoseTimer.resetTimer();
            
        }
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        telemetry.addData("trigger",gamepad2.right_trigger);
        telemetry.update();
        if (gamepad2.right_trigger > .2) { //transfer on/off (shoot)

            robotMotors.setIntakePower(intakePow);
            robotMotors.setTransferPower(tranferPow);
            isIntake = true;
            isTransfer = true;

        } else {

            robotMotors.setTransferPower(0);

            isTransfer = false;
        }

        if (buttonDebounceTimer.getElapsedTime() >= 500 && other_helpers.anyButtonPressed(gamepad2)) { //debounce all buttons

            if (gamepad2.right_bumper) {
                shooterSpeedAdjust += shooterSpeedAjustIncrement;
            }
            if (gamepad2.left_bumper) {
                shooterSpeedAdjust -= shooterSpeedAjustIncrement;
            }

            if (gamepad2.dpad_right) {
                turretPosAdjust += turretPosAdjustIncrement;
            }
            if (gamepad2.dpad_left) {
                turretPosAdjust -= turretPosAdjustIncrement;
            }

            if (gamepad2.a || gamepad2.y) { //intake on/off
                if (!isIntake) {
                    robotMotors.setIntakePower(intakePow);
                    robotMotors.setTransferPower(0);
                    isIntake = true;
                    isTransfer = false;
                } else {
                    robotMotors.setIntakePower(0);
                    robotMotors.setTransferPower(0);
                    isIntake = false;
                    isTransfer = false;
                }

            }
            buttonDebounceTimer.resetTimer();
        }

        if(use_PP) {
            // drive code...
            if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2) { //driver
                scalar = .5; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar - gamepad2.right_stick_x * scalar, 3), false);
            } else {
                scalar = 1.0; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 3), Math.pow(-gamepad1.right_stick_x * scalar - gamepad2.right_stick_x * scalar, 3), false);
            }


            follower.update();

            if (useFusion) {
                long now = System.nanoTime();
                fusion.pushOdomSample(now); // Push new odometry data for fusion

                if (poseTimer.getElapsedTime() >= 10) { //update servo tracking based on current pose every x ms
                    getCurBotPose();
                    Servos.pointTurretToGoal(curBotPoseX, curBotPoseY, curBotPoseHead, blueGoalX, blueGoalY, turretPosAdjust);
                    if ("blue".equals(myAllianceColor)) {
                        ll_goal_dist = other_helpers.distanceToBlueGoal(curBotPoseX, curBotPoseY);
                    } else {
                        ll_goal_dist = other_helpers.distanceToRedGoal(curBotPoseX, curBotPoseY);
                    }
                    double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(ll_goal_dist);
                    double speed = targetRPM * ticks_per_rev / 60;
                    robotMotors.setShooterVelocity(speed + shooterSpeedAdjust); // Set shooter velocity using the adjustment value from the gamepad
                    poseTimer.resetTimer();
                }

                if (llPoseTimer.getElapsedTime() >= 100) { // Fused pose update from Limelight
                    now = System.nanoTime();
                    fusion.updateFromLimelight(now);
                    llPoseTimer.resetTimer();
                }
            }
        }

        if(llTimer.getElapsedTime() >= 10 && use_LL) {
            LLResult result = limelight.limelight.getLatestResult();
            llTimer.resetTimer();

            if (result.isValid()) {
                target_acquired = true;
                Servos.setLedColor(servos.LedColor.GREEN);
                targetTimer.resetTimer();

                ll_goal_heading = headingAverage.updateAndGetAverage(result.getTx());
                ll_goal_dist = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());

                telemetry.addData("dist", ll_goal_dist);
                telemetry.addData("heading", ll_goal_heading);
            } else {
                Servos.setLedColor(servos.LedColor.RED);
                if (targetTimer.getElapsedTime()>=500){
                    target_acquired = false;
                    targetTimer.resetTimer();
                }
            }

            if(target_acquired) {
                // Set shooter velocity
                if (ll_goal_dist > 0.1 && ll_goal_dist < 4) {
                    double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(ll_goal_dist);
                    double speed = targetRPM * ticks_per_rev / 60;
                    robotMotors.setShooterVelocity(speed + shooterSpeedAdjust);
                    telemetry.addData("RPM", speed * 60 / ticks_per_rev);
                }

                // Adjust turret using PID
                if (Math.abs(ll_goal_heading) > 0.01) {
                    double pidOutput = Servos.updateTurretWithPID(ll_goal_heading / 500); // Pass current heading and target (0)
//            double pidOutput = headingPid.updatePID(ll_goal_heading, 0); // Pass current heading and target (0)
//            double pidOutput = ll_goal_heading / 500;

                    telemetry.addData("position", Servos.getTurretServoPos());
                    telemetry.addData("pidOutput", pidOutput);



                    telemetry.addData("position", Servos.getTurretServoPos());
                    telemetry.addData("heading_error_rad", ll_goal_heading / 500);
                }
                telemetry.addData("Target Acquired", target_acquired);
            }
            else {
                /*// Scan for the AprilTag
                double newScanPos = Servos.getTurretServoPos() + (0.0007 * scanCW);
                Servos.setTurretServoPos(newScanPos);
                if (Servos.getTurretServoPos() >= servos.TURRET_MAX_POS || Servos.getTurretServoPos() <= servos.TURRET_MIN_POS){
                    scanCW *= -1;
                }
                telemetry.addData("Target Acquired", target_acquired);

                 */
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
