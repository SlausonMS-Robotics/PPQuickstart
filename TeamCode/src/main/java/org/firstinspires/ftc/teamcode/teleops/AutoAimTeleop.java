package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.TurretAiming;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.sensors;
import org.firstinspires.ftc.teamcode.robot.servos;
import org.firstinspires.ftc.teamcode.robot.states;

@TeleOp(name = "Auto Aim Teleop", group = "23609")
public class AutoAimTeleop extends OpMode {

    private String myAllianceColor = "blue";

    private Timer buttonDebounceTimer;
    motors robotMotors = new motors();
    limelight3A limelight = new limelight3A();
    servos Servos = new servos();
    states robotStateController = new states();
    private TurretAiming turretAimer;

    private sensors mySensors;
    private int robotState = 0;
    private boolean useOdomTracking = true;
    private int previousRobotState = 0;

    Follower follower;

    @Override
    public void init() {
        robotMotors.init(hardwareMap);
        //mySensors.init(hardwareMap);
        Servos.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry);
        limelight.init(hardwareMap, 3, telemetry, Servos, follower);
        limelight.pollLimelight();




        buttonDebounceTimer = new Timer();

        Pose startPose;
        if (PoseStorage.autoFinished) {
            startPose = PoseStorage.currentPose;
            myAllianceColor = PoseStorage.allianceColor;
        } else {
            if("blue".equals(myAllianceColor)){
                startPose = new Pose(43.5,9,Math.toRadians(90));
                PoseStorage.allianceColor = "blue";
            } else {
                startPose = new Pose(100.5,9,Math.toRadians(90));
                PoseStorage.allianceColor = "red";
            }
        }
        follower.setStartingPose(startPose);
        telemetry.addData("Alliance Color", myAllianceColor);
        telemetry.addData("Starting Pose", startPose);
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // Handle state changes for shooting and intake
        if (gamepad1.right_trigger > .2) {
            robotState = 2;
        } else {
            robotState = previousRobotState;
        }

        double scalar = (gamepad1.left_trigger > .2) ? 0.5 : 1.0;

        if (buttonDebounceTimer.getElapsedTime() >= 500) {
            if (gamepad1.y) {
                if(robotState == 0) robotState = 1;
                else robotState = 0;
                buttonDebounceTimer.resetTimer();
            }
            if(gamepad1.x){
                //turretAimer.updatePoseFromLimelight();
                //telemetry.addData("IMU Yaw", mySensors.getImuYawDeg());
                robotMotors.setShooterVelocity(robotMotors.getShooterVelocity(6000));
                buttonDebounceTimer.resetTimer();
            }
            if(gamepad1.b){
                //turretAimer.updatePoseFromLimelight();
                //telemetry.addData("IMU Yaw", mySensors.getImuYawDeg());
                robotMotors.setShooterVelocity(robotMotors.getShooterVelocity(3000));
                buttonDebounceTimer.resetTimer();
            }
            if(gamepad1.a){
                //turretAimer.updatePoseFromLimelight();
                //telemetry.addData("IMU Yaw", mySensors.getImuYawDeg());
                robotMotors.setShooterVelocity(robotMotors.getShooterVelocity(2650));
                buttonDebounceTimer.resetTimer();
            }
        }
        if (robotState != 2) previousRobotState = robotState;

        if (buttonDebounceTimer.getElapsedTime() >= 150) {
            if (gamepad1.right_bumper){
                Servos.incrementTurretInDegrees(-5 * scalar);
                buttonDebounceTimer.resetTimer();
            }
            if (gamepad1.left_bumper){
                Servos.incrementTurretInDegrees(5 * scalar);
                buttonDebounceTimer.resetTimer();
            }
        }



        if (useOdomTracking) {
           // turretAimer.updateOdomAiming(myAllianceColor);
        }



        // Drive code

        follower.setTeleOpDrive(
            Math.pow(gamepad1.left_stick_x * scalar, 1),
            Math.pow(-gamepad1.left_stick_y * scalar, 1),
            Math.pow(-gamepad1.right_stick_x * scalar, 1),
            false);

        robotStateController.setRobotState(robotState, Servos, robotMotors);
        telemetry.addData("Bot Pose", follower.getPose());
        follower.update();
        telemetry.update();
    }

    @Override
    public void stop() {}
}
