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
import org.firstinspires.ftc.teamcode.robot.servos;
import org.firstinspires.ftc.teamcode.robot.states;

/**
 * This is an example TeleOp for testing out the 2360 robot. It'll need to be updated to allow for
 * targeting other AprilTags
 */
@TeleOp(name = "Auto Aim Teleop", group = "23609")
public class AutoAimTeleop extends OpMode {

    private String myAllianceColor = "blue";

    private Timer llPoseTimer, buttonDebounceTimer;
    motors robotMotors = new motors(); // Single object for all motors
    limelight3A limelight = new limelight3A();
    servos Servos = new servos();

    states robotStateController = new states();


    private TurretAiming turretAimer;
    private int robotState = 0;
    private int previousRobotState = 0;
    private int intakeState = 0;
    private int transferState = 0;
    private Pose startPose;


    Follower follower;



    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        // Initialize all our robot hardware
        robotMotors.init(hardwareMap); //initializes all motors
        Servos.init(hardwareMap, telemetry); // This now initializes the turret servo and PID

        follower = Constants.createFollower(hardwareMap);


        // Instantiate our new reusable classes
        turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry);

        llPoseTimer = new Timer();
        buttonDebounceTimer = new Timer();
        
        //myAllianceColor = PoseStorage.allianceColor;
        if("blue".equals(myAllianceColor)){
            //limelight.init(hardwareMap,0, telemetry);
            startPose = new Pose(44,9,Math.toRadians(90));

        }
        else {
            //limelight.init(hardwareMap,1, telemetry);
            startPose = new Pose(72,9,Math.toRadians(90));
        }
        follower.setStartingPose(startPose);
        telemetry.addData("Alliance Color", myAllianceColor);
        telemetry.addData("Starting Pose", startPose);
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {

            follower.startTeleopDrive(true);
            llPoseTimer.resetTimer();

    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {


        if (gamepad2.right_trigger > .2) { //transfer on/off (shoot)
            if (robotState != 2) previousRobotState = robotState;
            robotState = 2;

        }
        else robotState = previousRobotState;

        // Gamepad button logic for adjustments and intake
        if (buttonDebounceTimer.getElapsedTime() >= 350) { //debounce all buttons

            if (gamepad2.a || gamepad2.y) { //intake on/off
                if(robotState == 0) robotState = 1;
                else robotState = 0;
                buttonDebounceTimer.resetTimer();

            }
        }

        // Drive code
        double scalar;
        if (gamepad1.left_trigger > .2) { //driver
            scalar = .5; //sets speed of change -> lower = slower

        } else {
            scalar = 1.0; //sets speed of change -> lower = slower
        }
        follower.setTeleOpDrive(Math.pow(gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.left_stick_y * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar, 1), false);


        turretAimer.updateOdomAiming(myAllianceColor);
        robotStateController.setRobotState(robotState, Servos, robotMotors);
        follower.update();





    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
