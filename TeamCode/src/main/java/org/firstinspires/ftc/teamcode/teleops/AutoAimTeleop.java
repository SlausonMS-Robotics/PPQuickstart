package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.TurretAiming;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.other_helpers;
import org.firstinspires.ftc.teamcode.robot.sensors;
import org.firstinspires.ftc.teamcode.robot.servos;

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
    sensors mySensors = new sensors();
    private TurretAiming turretAimer;


    Follower follower;

    private int shooterSpeedAdjust = 0;
    private double turretPosAdjust = 0;

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        // Initialize all our robot hardware
        Servos.init(hardwareMap); // This now initializes the turret servo and PID
        robotMotors.init(hardwareMap, Servos); //initializes all motors and passes the servos object
        mySensors.init(hardwareMap); // Initialize the sensors
        myAllianceColor = PoseStorage.allianceColor;
        if("blue".equals(myAllianceColor)) {
            limelight.init(hardwareMap, 0, telemetry);
        }
        else {
            limelight.init(hardwareMap, 1, telemetry);
        }

        follower = Constants.createFollower(hardwareMap);
        Pose startPose = PoseStorage.currentPose;
        follower.setStartingPose(startPose);
        telemetry.addData("Starting Pose", startPose);

        // Instantiate our new reusable classes
        turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry);


        llPoseTimer = new Timer();
        buttonDebounceTimer = new Timer();
        

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
            } else {
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


            follower.startTeleopDrive(true);
            llPoseTimer.resetTimer();

    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        //telemetry.addData("dist", mySensors.getTransferDist(DistanceUnit.INCH));

        if (gamepad1.right_trigger > .2) { //transfer on/off (shoot)
            robotMotors.shoot();
        } else {
            if(robotMotors.isIntakeOn()) {
                robotMotors.transferBackwards(); //turn transfer off
            }
            else robotMotors.setTransfer(false);
        }
/*
        // Manual turret control with right joystick
        if (Math.abs(gamepad2.right_stick_x) > 0.1) {
            Servos.moveTurretManually(gamepad2.right_stick_x / 100);
            turretAimer.setManualTurret(true);
        }
        else {
            turretAimer.setManualTurret(false);
        }
*/
        // Gamepad button logic for adjustments and intake
        if (buttonDebounceTimer.getElapsedTime() >= 250) { //debounce all buttons
            if (other_helpers.anyButtonPressed(gamepad2) || gamepad1.a || gamepad1.y) {
                int shooterSpeedAdjustIncrement = 50;
                if (gamepad2.dpad_up) {
                    shooterSpeedAdjust += shooterSpeedAdjustIncrement;
                }
                if (gamepad2.dpad_down) {
                    shooterSpeedAdjust -= shooterSpeedAdjustIncrement;
                }

                double turretPosAdjustIncrement = .008;
                if (gamepad2.dpad_right) {
                    turretPosAdjust += turretPosAdjustIncrement;
                }
                if (gamepad2.dpad_left) {
                    turretPosAdjust -= turretPosAdjustIncrement;
                }

                if (gamepad1.a || gamepad1.y) { //intake on/off
                    robotMotors.toggleIntake(); //toggle intake
                }
                buttonDebounceTimer.resetTimer();
            }
        }

        // Pass manual adjustments to the TurretAiming class
        if (turretAimer != null) {
            turretAimer.setShooterSpeedAdjust(shooterSpeedAdjust);
            turretAimer.setTurretPosAdjust(turretPosAdjust);
        }


            // Drive code
            double scalar;
            if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2) { //driver
                scalar = .5; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 1), Math.pow(-gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar, 1), false);
            } else {
                scalar = 1.0; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 1), Math.pow(-gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar, 1), false);
            }
            
            // Update odometry and sensor fusion
            follower.update();

            // Update turret aiming logic
            if (turretAimer != null){
                if (!turretAimer.isManualTurret()) {
                    turretAimer.update(myAllianceColor);
                }
            }

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }

}
