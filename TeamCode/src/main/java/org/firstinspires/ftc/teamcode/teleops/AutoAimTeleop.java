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
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */
//wifi direct password = 4EaUU37n

@TeleOp(name = "Auto Aim Teleop", group = "23609")
public class AutoAimTeleop extends OpMode {
    // With a 5-turn servo, the P gain needs to be much smaller. Start here for tuning.
    public static double SERVO_P = 0.015, SERVO_I = 0.0, SERVO_D = 0.005;
    private static final int MOVING_AVERAGE_SIZE = 3;
    private static double scalar = 1.0;
    private static int ticks_per_rev = 28;

    private double ll_goal_dist = 0;
    private double ll_goal_heading = 0;
    private Timer targetTimer, llTimer;
    motors shooter = new motors();
    limelight3A limelight = new limelight3A();
    // Hardware that we're using
    private ServoImplEx turretServo;     // servo0

    // PID and Moving Average Helpers
    private other_helpers headingPid = new other_helpers();
    private other_helpers headingAverage = new other_helpers();
    private other_helpers distanceAverage = new other_helpers();

    Follower follower;
    private boolean use_PP = false;
    private final Pose startPose = new Pose(0,0,0);

    private double lastTime = 0.0;

    double turretPos = 0.0;
    private boolean target_acquired = false;

    private double turretPosMax = .75;
    private double turretPosMin = .25;
    private int scanCW = 1;
    private double lastCorrection = 0;


    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        if (use_PP) {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);
        }
        // Initialize the servo that controls the turrent and center it.
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

//        // Testing the servo
//        double turretPos = turretServo.getPosition() + 0.05;
//        if (turretPos > 0.75) {
//            turretPos = 0.25;
//        }
//        turretServo.setPosition(turretPos);
//        telemetry.addData("Turrent Pos", turretPos);
//        telemetry.update();
//        if (true) {
//            try {
//                Thread.sleep(500);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            return;
//        }

//        try {
//            Thread.sleep(500);
//        }
//        catch (InterruptedException ex) {
//            // noop
//        }
        if(llTimer.getElapsedTime() >= 10) {
            LLResult result = limelight.limelight.getLatestResult();
            llTimer.resetTimer();
            // Skip if no updates

            if (result.isValid()) {
                target_acquired = true;
                targetTimer.resetTimer();

                ll_goal_heading = headingAverage.updateAndGetAverage(result.getTx());
                ll_goal_dist = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());

                telemetry.addData("dist", ll_goal_dist);
                telemetry.addData("heading", ll_goal_heading);
            } else {

                if (targetTimer.getElapsedTime()>=650){ //if the limelight hasn't seen the Apriltag for this many ms
                    target_acquired = false;
                    targetTimer.resetTimer();
                }
            }

            if(target_acquired) {

                //double shooter_velocity_scalar = 4000.0 / 60.0 / 3.3; //(Revs / second) / Meter
                // Set shooter velocity based on distance, not heading
                if (ll_goal_dist > 0.1 && ll_goal_dist < 4) {
                    double targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(ll_goal_dist);
                    double speed = targetRPM * ticks_per_rev / 60;
                    shooter.setShooterVelocity(speed);
                    telemetry.addData("RPM", speed * 60 / ticks_per_rev);
                }

                if (Math.abs(ll_goal_heading) > 0.01) { // Only adjust if we are off-target
                    double pidOutput = headingPid.updatePID(ll_goal_heading / 500, 0); // Pass current heading and target (0)
//            double pidOutput = headingPid.updatePID(ll_goal_heading, 0); // Pass current heading and target (0)
//            double pidOutput = ll_goal_heading / 500;

                    telemetry.addData("position", turretServo.getPosition());
                    telemetry.addData("pidOutput", pidOutput);

                    pidOutput = Math.min(Math.max(pidOutput, -0.01), 0.01);
                    if(pidOutput >= 0){
                        scanCW = 1;
                    }
                    else {
                        scanCW = -1;
                    }
//            pidOutput = Math.min(Math.max(pidOutput, -0.001), 0.001);
                    telemetry.addData("Clamped pidOutput", pidOutput);

                    //
//            double turretPosition = turretServo.getPosition() - pidOutput;
                    // Needed for headingPid above
                    double turretPosition = turretServo.getPosition() + pidOutput;

                    if (turretPosition < turretPosMin) {
                        turretPosition = turretPosMin;
                        telemetry.addData("Clamping", turretPosMin);
                    }
                    if (turretPosition > turretPosMax) {
                        turretPosition = turretPosMax;
                        telemetry.addData("Clamping", turretPosMax);
                    }

                    telemetry.addData("Turret Pos", turretPosition);
                    turretServo.setPosition(turretPosition);
                }
                telemetry.addData("Target Acquired", target_acquired);
            }
            else {
                turretServo.setPosition(Math.min(Math.max(turretServo.getPosition() + (.001 * scanCW),turretPosMin),turretPosMax));
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
