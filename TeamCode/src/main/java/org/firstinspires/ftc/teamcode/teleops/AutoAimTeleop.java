package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
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
    private static final double transferPow = 1.0;

    private boolean isIntake = false;
    private String myAllianceColor = "blue";
    private static final int MOVING_AVERAGE_SIZE = 3;
    private static final int ticks_per_rev = 28;

    private double llGoalDist = 0;
    private double llGoalHeadingError = 0;
    private Timer targetTimer, llTimer, poseTimer, llPoseTimer, buttonDebounceTimer;
    motors robotMotors = new motors(); // Single object for all motors
    limelight3A limelight = new limelight3A();
    servos Servos = new servos(); // Use the servos class

    // Moving Average Helpers
    private final other_helpers headingAverage = new other_helpers();
    private final other_helpers distanceAverage = new other_helpers();


    Follower follower;
    private final boolean usePP = true; // Enabled Pedro Pathing
    private final boolean useLL = true;
    private final Pose currentPose = new Pose(0,0,0);
    private double speed;

    double curBotPoseX = 0;
    double curBotPoseY = 0;
    double curBotPoseHead = 0;
    private int shooterSpeedAdjust = 0;
    private double turretPosAdjust = 0;
    private boolean target_acquired = false;


        /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        // Initialize all our robot hardware
        robotMotors.init(hardwareMap); //initializes all motors
        Servos.init(hardwareMap); // This now initializes the turret servo and PID
        limelight.init(hardwareMap,0, telemetry);

        if (usePP) {
            follower = Constants.createFollower(hardwareMap);
            Pose startPose = PoseStorage.currentPose;
            follower.setStartingPose(startPose);
            telemetry.addData("Starting Pose", startPose);
            telemetry.update();
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
            PoseStorage.allianceColor = myAllianceColor;
            
        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        if(usePP) {
            follower.startTeleopDrive(true);
            poseTimer.resetTimer();
            llPoseTimer.resetTimer();
            
        }
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        telemetry.addData("trigger", gamepad2.right_trigger);
        telemetry.update();
        if (gamepad2.right_trigger > .2) { //transfer on/off (shoot)

            robotMotors.setIntakePower(intakePow);
            robotMotors.setTransferPower(transferPow);
            isIntake = true;

        } else {

            robotMotors.setTransferPower(0);

        }

        if (buttonDebounceTimer.getElapsedTime() >= 500 && other_helpers.anyButtonPressed(gamepad2)) { //debounce all buttons

            int shooterSpeedAdjustIncrement = 50;
            if (gamepad2.right_bumper) {
                shooterSpeedAdjust += shooterSpeedAdjustIncrement;
            }
            if (gamepad2.left_bumper) {
                shooterSpeedAdjust -= shooterSpeedAdjustIncrement;
            }

            double turretPosAdjustIncrement = .008;
            if (gamepad2.dpad_right) {
                turretPosAdjust += turretPosAdjustIncrement;
            }
            if (gamepad2.dpad_left) {
                turretPosAdjust -= turretPosAdjustIncrement;
            }

            if (gamepad2.a || gamepad2.y) { //intake on/off
                if (!isIntake) {
                    robotMotors.setIntakePower(intakePow);
                    robotMotors.setTransferPower(-.05);
                    isIntake = true;
                } else {
                    robotMotors.setIntakePower(.05);
                    robotMotors.setTransferPower(0);
                    isIntake = false;
                }

            }
            buttonDebounceTimer.resetTimer();
        }

        if (usePP) {
            // drive code...
            double scalar;
            if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2) { //driver
                scalar = .5; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar - gamepad2.right_stick_x * scalar, 3), false);
            } else {
                scalar = 1.0; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 3), Math.pow(-gamepad1.right_stick_x * scalar - gamepad2.right_stick_x * scalar, 3), false);
            }


            follower.update();



            if (llTimer.getElapsedTime() >= 10 && useLL) {
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
                    goalDistance = other_helpers.distanceToGoal(follower.getPose().getX(), follower.getPose().getY());
                    goalHeadingError = other_helpers.getHeadingErrorToGoal(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading());


                }
                // Set shooter velocity
                if (goalDistance > 0 && goalDistance < 5) {
                    double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(goalDistance);
                    speed = targetRPM * ticks_per_rev / 60;
                    robotMotors.setShooterVelocity(speed + shooterSpeedAdjust);

                }

                // Adjust turret using PID
                if (Math.abs(goalHeadingError) > 0.0) {
                    Servos.updateTurretWithPID(goalHeadingError / 500); // Pass current heading and target (0)

                }
                telemetry.addData("Target Acquired?", target_acquired);
                telemetry.addData("RPM", speed * 60 / ticks_per_rev);
                telemetry.addData("Goal Distance", goalDistance);
                telemetry.addData("Goal Heading Error", goalHeadingError);
                telemetry.addData("Current Pose", follower.getPose());
                telemetry.update();
            }
        }
    }
    public void getCurBotPose() {
        if (usePP) {
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
