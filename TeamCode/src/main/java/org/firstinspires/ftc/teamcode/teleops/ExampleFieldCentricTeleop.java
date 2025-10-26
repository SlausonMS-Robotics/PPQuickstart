package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.robot.servos.WRIST_FULL_EXTENSION_POS;
import static org.firstinspires.ftc.teamcode.robot.servos.WRIST_FULL_RETRACTION_POS;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.servos;
/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */

@TeleOp(name = "Example Field-Centric Teleop", group = "Examples")
public class ExampleFieldCentricTeleop extends OpMode {
    private Follower follower;
    private static double scalar = 1.0;
    private double xval = 0;
    private double yval = 0;
    private double hval = 0;
    private Timer myTimer, llTimer;
    servos robotservo = new servos();
    servos wristServo = new servos();
    servos grabServo = new servos();

    motors shooter = new motors();
    //limelight3A limelight = new limelight3A();

    private int state = 0;
    private boolean gripOpen = false;

    private boolean runState = false;
    private boolean precise = false;

    private final Pose startPose = new Pose(0,0,0);

    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robotservo.init(hardwareMap);
        shooter.init(hardwareMap);
        //limelight.init(hardwareMap,5, follower, telemetry);
        myTimer = new Timer();
        llTimer = new Timer();
        //robotservo.headlightOn();

    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive(false);
        robotservo.openGripper();
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {

        /* Update Pedro to move the robot based on:
        - Forward/Backward Movement: -gamepad1.left_stick_y
        - Left/Right Movement: -gamepad1.left_stick_x
        - Turn Left/Right Movement: -gamepad1.right_stick_x
        - Robot-Centric Mode: false
        */

        follower.update();

        shooter.shooterPower(1);

        if(gamepad1.left_trigger > .1) {
            scalar = .5;
            follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar,1), Math.pow(-gamepad1.left_stick_x * scalar,1), Math.pow(-gamepad1.right_stick_x * scalar,1), false);
        }
        else {
            scalar = 1.0;
            follower.setTeleOpDrive(Math.pow(-gamepad1.left_stick_y * scalar,3), Math.pow(-gamepad1.left_stick_x * scalar,3), Math.pow(-gamepad1.right_stick_x * scalar,3), false);
        }

        if(gamepad1.right_trigger > .1) {
            shooter.shooterPower(gamepad1.right_trigger);
        }
        else {
            shooter.shooterPower(0);
        }

        follower.update();

        if(gamepad1.b && myTimer.getElapsedTime() > 300) {
            runState = true;
            myTimer.resetTimer();
            state++;
        }
        if(gamepad1.x && myTimer.getElapsedTime() > 300) {
            runState = true;
            myTimer.resetTimer();
            state--;
            if (state < 0){
                state = 0;
            }
        }

        if (runState){
            switch (state) {
                case 0:
                    robotservo.slideFullRetract();
                    robotservo.setWristServo(WRIST_FULL_RETRACTION_POS);

                    break;
                case 1:
                    robotservo.slideFullExtend();
                    break;
                case 2:
                    robotservo.setWristServo(.5);
                    break;
                case 3:
                    robotservo.setWristServo(WRIST_FULL_EXTENSION_POS);
                    break;
                case 4:
                    robotservo.closeGripper();
                    state = 3;
                    break;

            }
            runState = false;
        }

        if(gamepad1.y && myTimer.getElapsedTime() > 300){
            if (gripOpen) {
                robotservo.closeGripper();
                gripOpen = false;
                myTimer.resetTimer();
            }
            else {
                robotservo.openGripper();
                gripOpen = true;
                myTimer.resetTimer();
            }
        }
/*
        if (gamepad1.right_trigger > .1) {
            if (!limelight.getLLStatus()) limelight.startLL(100);
            boolean llGood = limelight.pollLimelight();
            if (llGood) {
                double targetDist = limelight.getXDist(0);
                if (targetDist > 0) robotservo.setSlideInches(targetDist);
            }
        }

 */
/*
        if(gamepad1.a && !follower.isBusy()) {
            if (!limelight.getLLStatus()) limelight.startLL(200);
            if (limelight.pollLimelight()) {
                //limelight.LLDriveTo();
                follower.followPath(limelight.LLDriveTo());
                follower.update();
            }
        }

        if (llTimer.getElapsedTimeSeconds() >= 3 && limelight.getLLStatus()) {
            limelight.stopLL();
            llTimer.resetTimer();
            follower.startTeleopDrive();
        }
*/


        /* Telemetry Outputs of our Follower
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Xdist",limelight.getXDist(0));
        //telemetry.addData("SPos", slide_pos);
        telemetry.addData("Xdeg",limelight.getXDeg(0));
        telemetry.addData("Ydeg",limelight.getYDeg(0));
        telemetry.addData("area",limelight.getArea(0));
        /* Update Telemetry to the Driver Hub */
        //telemetry.update();

    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
        robotservo.slideServoOff();
    }

}