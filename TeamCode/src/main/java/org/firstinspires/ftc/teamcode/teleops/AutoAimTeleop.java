package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.TurretAiming;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.other_helpers;
import org.firstinspires.ftc.teamcode.robot.sensors;
import org.firstinspires.ftc.teamcode.robot.servos;
import org.firstinspires.ftc.teamcode.robot.states;

@TeleOp(name = "Auto Aim Teleop", group = "23609")
public class AutoAimTeleop extends OpMode {

    private String myAllianceColor = "blue";
    private other_helpers helpers;
    private Timer buttonDebounceTimer;
    motors robotMotors = new motors();

    sensors Sensors = new sensors();
    limelight3A limelight = new limelight3A();
    servos Servos = new servos();
    states robotStateController = new states();
    private TurretAiming turretAimer;

    private int robotState = 0;
    private boolean useOdomTracking = true;


    private int previousRobotState = 0;
    private int buttonCount = 0;
    static double FAR_BOUNCE_POS = .33;
    static double MID_BOUNCE_POS = .42;
    static double CLOSE_BOUNCE_POS = .52;
    static int FAR_SHOOTER_RPM = 3300;
    static int MID_SHOOTER_RPM = 2900;
    static int CLOSE_SHOOTER_RPM = 2600;

    private boolean shot = false;

    Follower follower;


    @Override
    public void init() {
        robotMotors.init(hardwareMap);
        Sensors.init(hardwareMap);
        Servos.init(hardwareMap, telemetry);
        follower = Constants.createFollower(hardwareMap);
        turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry, helpers);
        limelight.init(hardwareMap, 3, telemetry, Servos, follower);
        limelight.pollLimelight();
        Servos.setBouncerServo(.5);




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
            shot = true;
        } else {
            robotState = previousRobotState;
        }

        double scalar = (gamepad1.left_trigger > .2) ? 1.2 : .25;


        if (Sensors.getCSDistanceMM() < 50 && robotState != 2){

                robotState = 3;
                shot = false;

        }
        else if(robotState != 2 && shot) {
            robotState = 1;
        }

        if(gamepad1.start){
            robotMotors.setShooterVelocity(0);
        }

        if(gamepad1.back) {
            robotMotors.setShooterVelocity(robotMotors.getShooterVelocityFromRPM(6000));
        }

        if (buttonDebounceTimer.getElapsedTime() >= 500) {
            if (gamepad1.y) {
                if(robotState == 0) robotState = 1;
                else robotState = 0;
                buttonDebounceTimer.resetTimer();
            }
            if(gamepad1.x){

                turretAimer.setShooter(1.25); // sets rpm and bouncer position to x meters

                buttonDebounceTimer.resetTimer();
            }

            if(gamepad1.dpad_up){
                    turretAimer.LLAim();

            }
            //else turretAimer.llAim(false);

            if(gamepad1.b){

                turretAimer.setShooter(3.2); // sets rpm and bouncer position to x meters
                buttonDebounceTimer.resetTimer();
            }
            if(gamepad1.a){
                //turretAimer.updatePoseFromLimelight();
                //telemetry.addData("IMU Yaw", Sensors.getImuYawDeg());
                turretAimer.setShooter(2.25); // sets rpm and bouncer position to x meters
                buttonDebounceTimer.resetTimer();
            }
            if(gamepad1.dpad_left){
                double pos = Servos.getBouncerServoPos();
                pos -= .01;
                Servos.setBouncerServo(pos);
                buttonDebounceTimer.resetTimer();
            }
            if(gamepad1.dpad_right){
                double pos = Servos.getBouncerServoPos();
                pos += .01;
                Servos.setBouncerServo(pos);
                buttonDebounceTimer.resetTimer();
            }
        }
        if (robotState != 2) previousRobotState = robotState;

        if (buttonDebounceTimer.getElapsedTime() >= 150) {
            if (gamepad1.right_bumper){
                Servos.incrementTurretInDegrees(-3 * scalar);
                buttonDebounceTimer.resetTimer();
            }
            if (gamepad1.left_bumper){
                Servos.incrementTurretInDegrees(3 * scalar);
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
        follower.update();

        robotStateController.setRobotState(robotState, Servos, robotMotors);
        telemetry.addData("Bot Pose", follower.getPose());
        telemetry.addData("Bouncer Pos", Servos.getBouncerServoPos());
        telemetry.addData("Sensor Dist", Sensors.getCSDistanceMM());
        telemetry.update();
    }

    @Override
    public void stop() {}
}
