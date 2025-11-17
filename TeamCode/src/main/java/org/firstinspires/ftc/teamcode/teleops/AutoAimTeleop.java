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

    private Timer llPoseTimer, buttonDebounceTimer;
    motors robotMotors = new motors(); // Single object for all motors
    limelight3A limelight = new limelight3A();
    servos Servos = new servos();
    private TurretAiming turretAimer;


    Follower follower;
    private final boolean usePP = true; // Enabled Pedro Pathing

    private int shooterSpeedAdjust = 0;
    private double turretPosAdjust = 0;

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

            // Instantiate our new reusable classes
            turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry);
        }

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
        if(usePP) {
            follower.startTeleopDrive(true);
            llPoseTimer.resetTimer();
        }
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        // Intake and transfer logic
        if (gamepad2.right_trigger > .2) { //transfer on/off (shoot)
            robotMotors.setIntakePower(intakePow);
            robotMotors.setTransferPower(transferPow);
            isIntake = true;
        } else {
            robotMotors.setTransferPower(0);
        }

        // Gamepad button logic for adjustments and intake
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

        // Pass manual adjustments to the TurretAiming class
        if (turretAimer != null) {
            turretAimer.setShooterSpeedAdjust(shooterSpeedAdjust);
            turretAimer.setTurretPosAdjust(turretPosAdjust);
        }

        if (usePP) {
            // Drive code
            double scalar;
            if (gamepad1.left_trigger > .2 || gamepad2.left_trigger > .2) { //driver
                scalar = .5; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 1), Math.pow(-gamepad1.right_stick_x * scalar - gamepad2.right_stick_x * scalar, 3), false);
            } else {
                scalar = 1.0; //sets speed of change -> lower = slower
                follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar, 3), Math.pow(-gamepad1.left_stick_x * scalar, 3), Math.pow(-gamepad1.right_stick_x * scalar - gamepad2.right_stick_x * scalar, 3), false);
            }
            
            // Update odometry and sensor fusion
            follower.update();


            // Update turret aiming logic
            if (turretAimer != null) {
                turretAimer.update(myAllianceColor);
            }
        }
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}
