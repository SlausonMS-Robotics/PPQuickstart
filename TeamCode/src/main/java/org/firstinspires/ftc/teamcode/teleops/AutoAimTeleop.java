package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.other_helpers;

/**
 * This is an example TeleOp for testing out the 2360 robot. It'll need to be updated to allow for
 * targeting other AprilTags
 */
@TeleOp(name = "Auto Aim Teleop", group = "23609")
public class AutoAimTeleop extends OpMode {
    // With a 5-turn servo, the P gain needs to be much smaller. Start here for tuning.
    public static double SERVO_P = 0.015, SERVO_I = 0.0, SERVO_D = 0.005;
    private static final int MOVING_AVERAGE_SIZE = 3;
    private static double scalar = 1.0;
    private static final int ticks_per_rev = 28;

    private double ll_goal_dist = 0;
    private double ll_goal_heading = 0;
    private Timer targetTimer, llTimer;
    motors shooter = new motors();
    limelight3A limelight = new limelight3A();
    // Hardware that we're using
    private ServoImplEx turretServo;    // servo0

    // PID and Moving Average Helpers
    private other_helpers headingPid = new other_helpers();
    private other_helpers headingAverage = new other_helpers();
    private other_helpers distanceAverage = new other_helpers();

    Follower follower;
    private boolean use_PP = false;
    private final Pose startPose = new Pose(0,0,0);

    double turretPos = 0.0;
    private boolean target_acquired = false;

    private final double turretPosMax = .75;
    private final double turretPosMin = .25;
    private int scanCW = 1;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        if (use_PP) {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);
        }
        // Initialize the servo that controls the turret and center it.
        turretServo = hardwareMap.get(ServoImplEx .class, "servo0");
        turretServo.setPosition(0.5);

        shooter.init(hardwareMap);
        limelight.init(hardwareMap,0, telemetry);

        headingPid.initPID(SERVO_P, SERVO_I, SERVO_D);
        headingAverage.initMovingAverage(MOVING_AVERAGE_SIZE);
        distanceAverage.initMovingAverage(MOVING_AVERAGE_SIZE);

        targetTimer = new Timer();
        llTimer = new Timer();
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
        }
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        if(use_PP) {
            if (gamepad1.left_trigger > .1) {
                scalar = .5;
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 1), Math.pow(-gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar, 1), false);
            } else {
                scalar = 1.0;
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 3), Math.pow(-gamepad1.right_stick_x * scalar, 3), false);
            }
            follower.update();
        }

        // Check the Limelight for new data every 10ms. Otherwise, skip this check which lets the
        // robot continue doing whatever else it needs to.
        if(llTimer.getElapsedTime() >= 10) {
            LLResult result = limelight.limelight.getLatestResult();
            llTimer.resetTimer();

            // Limelight may not have a new result ready. If not, it'll return isValid() == false
            if (result.isValid()) {
                // A result means that the AprilTag of interest can be found. We track that the
                // target is acquired so that the auto-scanning code doesn't run.
                target_acquired = true;
                targetTimer.resetTimer();

                // The main things returned by Limelight pipeline that looks for one AprilTag are
                // x-axis, y-axis and an estimated average distance to the AprilTag detected.
                // ll_goal_heading is how far off in the x-axis the AprilTag is (e.g. left or right)
                // ll_goal_dist is the estimated average distance to the AprilTag
                ll_goal_heading = headingAverage.updateAndGetAverage(result.getTx());
                ll_goal_dist = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());

                telemetry.addData("dist", ll_goal_dist);
                telemetry.addData("heading", ll_goal_heading);
            } else {
                // If we haven't see the AprilTag in awhile, start auto-scanning to try and find it.
                // The cutoff here is in milliseconds.
                if (targetTimer.getElapsedTime()>=650){
                    target_acquired = false;
                    targetTimer.resetTimer();
                }
            }

            // When the target is visible we need to do two high-level things
            // 1. The flywheels are spinning at the right speed to reach the goal.
            // 2. The turret is aimed at the AprilTag
            if(target_acquired) {
                // Set shooter velocity based on distance
                if (ll_goal_dist > 0.1 && ll_goal_dist < 4) {
                    double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(ll_goal_dist);
                    double speed = targetRPM * ticks_per_rev / 60;
                    shooter.setShooterVelocity(speed);
                    telemetry.addData("RPM", speed * 60 / ticks_per_rev);
                }

                // Adjust the turret to aim at the AprilTag. Only do this if we're far enough off.
                // The cutoff makes it so that this code is skipped, if we're close to aimed correctly.
                if (Math.abs(ll_goal_heading) > 0.01) {
                    // Proportional-Integral-Derivative (PID) Controller is used to aim the turret.
                    // See https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
                    double pidOutput = headingPid.updatePID(ll_goal_heading / 500, 0); // Pass current heading and target (0)

                    telemetry.addData("position", turretServo.getPosition());
                    telemetry.addData("pidOutput", pidOutput);

                    // Changes to the turret are limited to avoid too big of a change.
                    pidOutput = Math.min(Math.max(pidOutput, -0.01), 0.01);
                    if(pidOutput >= 0){
                        scanCW = 1;
                    }
                    else {
                        scanCW = -1;
                    }
                    telemetry.addData("Clamped pidOutput", pidOutput);

                    // Calculate the updated turret position.
                    double turretPosition = turretServo.getPosition() + pidOutput;
                    // Clamp how far the turret can go in either direction. The servo can go farther
                    // but the cords connecting everything can't stretch that far.
                    if (turretPosition < turretPosMin) {
                        turretPosition = turretPosMin;
                        telemetry.addData("Clamping", turretPosMin);
                    }
                    if (turretPosition > turretPosMax) {
                        turretPosition = turretPosMax;
                        telemetry.addData("Clamping", turretPosMax);
                    }
                    // Finally, update the position of the turret
                    telemetry.addData("Turret Pos", turretPosition);
                    turretServo.setPosition(turretPosition);
                }
                telemetry.addData("Target Acquired", target_acquired);
            }
            else {
                // This does the scanning for when the AprilTag can't be found by the pipeline.
                // The code will move the turret all the way from side-to-side until it sees the tag.
                turretServo.setPosition(Math.min(Math.max(turretServo.getPosition() + (.0007 * scanCW),turretPosMin),turretPosMax));
                if (turretServo.getPosition() >= turretPosMax || turretServo.getPosition() <= turretPosMin){
                    scanCW *= -1;
                }
                telemetry.addData("Target Acquired", target_acquired);
            }
            telemetry.update();
        }
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
