package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.other_helpers;
import org.firstinspires.ftc.teamcode.robot.servos;

/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */
//wifi direct password = 4EaUU37n

@TeleOp(name = "Field-Centric Auto Aim Teleop", group = "Examples")
public class FieldCentricTeleopLLMegaTags extends OpMode {
    // With a 5-turn servo, the P gain needs to be much smaller. Start here for tuning.
    public static double SERVO_P = 0.002, SERVO_I = 0.0, SERVO_D = 0.0;
    private static final int MOVING_AVERAGE_SIZE = 5;
    private static double scalar = 1.0;
    private static int ticks_per_rev = 28;

    private double ll_goal_dist = 0;
    private double ll_goal_heading = 0;
    private Timer myTimer, llTimer;
    private servos Servos;
    servos turretServo = new servos();
    motors shooter = new motors();
    limelight3A limelight = new limelight3A();

    // PID and Moving Average Helpers
    private other_helpers headingPid = new other_helpers();
    private other_helpers headingAverage = new other_helpers();
    private other_helpers distanceAverage = new other_helpers();

    Follower follower;
    private double turretPosition = 0;
    private boolean use_PP = false;
    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        if (use_PP) {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);
        }
        turretServo.init(hardwareMap);
        shooter.init(hardwareMap, Servos);
        limelight.init(hardwareMap,0, telemetry);

        headingPid.initPID(SERVO_P, SERVO_I, SERVO_D);
        headingAverage.initMovingAverage(MOVING_AVERAGE_SIZE);
        distanceAverage.initMovingAverage(MOVING_AVERAGE_SIZE);

        myTimer = new Timer();
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

        LLResult result = limelight.limelight.getLatestResult();
        //if(limelight.pollLimelight()) {
        telemetry.addData("results is", result);
        if(result.isValid()) {
            ll_goal_heading = headingAverage.updateAndGetAverage(result.getTx());
            ll_goal_dist = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());

            telemetry.addData("dist", ll_goal_dist);
            telemetry.addData("heading", ll_goal_heading);
            telemetry.addData("results", result);
        }
        else {
            // Clear readings if we don't see a target
            ll_goal_dist = 0;
            ll_goal_heading = 0;
            headingAverage.clearReadings();
            distanceAverage.clearReadings();
        }

        double shooter_velocity_scalar = 5600.0 / 60.0 / 3.79; //(Revs / second) / Meter
        // Set shooter velocity based on distance, not heading
        if (ll_goal_dist > 0.1 && ll_goal_dist < 4){
            shooter.setShooterVelocity(shooter_velocity_scalar * ticks_per_rev * ll_goal_dist);
        }

        if(Math.abs(ll_goal_heading) > 0.1){ // Only adjust if we are off-target
            double pidOutput = headingPid.updatePID(ll_goal_heading, 0); // Pass current heading and target (0)
            turretPosition -= pidOutput;
            
            turretServo.setTurretServoPos(turretPosition);
        }

        telemetry.update();
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
