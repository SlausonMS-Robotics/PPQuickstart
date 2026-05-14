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
    private Timer buttonDebounceTimer, intakeCurrentTimer;
    private Timer llPollTimer;
    private Timer lockTimer;
    motors robotMotors = new motors();

    sensors Sensors = new sensors();
    limelight3A limelight = new limelight3A();
    servos Servos = new servos();
    states robotStateController = new states();
    private TurretAiming turretAimer;

    private int robotState = 1;
    private boolean aimSwitch = true;
    private boolean useOdomTracking = true;


    private int previousRobotState = 1;
    private int buttonCount = 0;
    static double FAR_BOUNCE_POS = .33;
    static double MID_BOUNCE_POS = .42;
    static double CLOSE_BOUNCE_POS = .52;
    static int FAR_SHOOTER_RPM = 3300;
    static int MID_SHOOTER_RPM = 2900;
    static int CLOSE_SHOOTER_RPM = 2600;

    private boolean shot = false;
    private boolean lock = false;

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
        //Servos.setBouncerServo(.5);




        buttonDebounceTimer = new Timer();
        llPollTimer = new Timer();
        lockTimer = new Timer();
        intakeCurrentTimer = new Timer();

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
    public void start() {

        follower.startTeleopDrive(true);
        robotState = 1;
        previousRobotState = 1;
        turretAimer.setShooter(2.15);
        turretAimer.zeroTurret();

    }

    @Override
    public void loop() {
        double scalar = (gamepad1.left_trigger > .2) ? 1.3 : .3;
        //robotState = 1;
        if (gamepad1.y) { //intake control
                /*
                if(robotState == 0) robotState = 1;
                else robotState = 0;
                buttonDebounceTimer.resetTimer();
                 */
            robotMotors.setIntakePower(-1);

        }
        else {

            if (robotMotors.getIntakeCurrent() <= 1 && robotState != 4) {
                intakeCurrentTimer.resetTimer();
                robotState = 1;

            } else if (intakeCurrentTimer.getElapsedTime() > 1000 ) {
                robotState = 4; //turn off intake if intake motor current is high for longer than x time
                gamepad1.rumble(1000); // rumble for 0.5 seconds
                previousRobotState = robotState;
                intakeCurrentTimer.resetTimer();
            }



            // Handle state changes for shooting and intake
            if (gamepad1.right_trigger > .2) {
                if (lock || lockTimer.getElapsedTime() > 750) {
                    robotState = 2;
                    shot = true;
                }
            } else {
                robotState = previousRobotState;
                if (lockTimer.getElapsedTime() > 750) {
                    lockTimer.resetTimer();
                }
                shot = false;
            }




            if (Sensors.getCSDistanceMM() < 50 && robotState == 1) {

                robotState = 3;
                shot = false;

            }


            if (gamepad1.start) {
                robotMotors.setShooterVelocity(0);
            }

            if (gamepad1.back) {
                //robotMotors.setShooterVelocity(robotMotors.getShooterVelocityFromRPM(6000));
            }

            if (buttonDebounceTimer.getElapsedTime() >= 300) {

                if (gamepad1.x) {

                    turretAimer.setShooter(1.2); // sets rpm and bouncer position to x meters

                    buttonDebounceTimer.resetTimer();
                }

                if (gamepad1.dpad_up) {
                    //aimSwitch = !aimSwitch;


                }

                //else turretAimer.llAim(false);

                if (gamepad1.b) {

                    turretAimer.setShooter(3.2); // sets rpm and bouncer position to x meters
                    buttonDebounceTimer.resetTimer();
                }
                if (gamepad1.a) {
                    //turretAimer.updatePoseFromLimelight();
                    //telemetry.addData("IMU Yaw", Sensors.getImuYawDeg());
                    turretAimer.setShooter(2.15); // sets rpm and bouncer position to x meters
                    buttonDebounceTimer.resetTimer();
                }
                if (gamepad1.dpad_left) {
                    double pos = Servos.getBouncerServoPos();
                    pos -= .01;
                    Servos.setBouncerServo(pos);
                    buttonDebounceTimer.resetTimer();
                }
                if (gamepad1.dpad_right) {
                    double pos = Servos.getBouncerServoPos();
                    pos += .01;
                    Servos.setBouncerServo(pos);
                    buttonDebounceTimer.resetTimer();
                }
            }


            if (buttonDebounceTimer.getElapsedTime() >= 100) {
                if (gamepad1.left_bumper) {
                    robotState = 1;
                }
                else if (gamepad1.right_bumper) {
                    robotState = 0;
                }
            }


            if (aimSwitch) {
                lock = turretAimer.LLAim(true);
                llPollTimer.resetTimer();
            } else lock = turretAimer.LLAim(false);
            robotStateController.setRobotState(robotState, Servos, robotMotors);
            if (robotState != 2 && robotState != 4) previousRobotState = robotState;

        }

        // Drive code

        follower.setTeleOpDrive(
            Math.pow(gamepad1.left_stick_x * scalar, 1),
            Math.pow(-gamepad1.left_stick_y * scalar, 1),
            Math.pow(-gamepad1.right_stick_x * scalar, 1),
            false);
        follower.update();


        //telemetry.addData("Bot Pose", follower.getPose());
        telemetry.addData("Bouncer Pos", Servos.getBouncerServoPos());
        telemetry.addData("Sensor Dist", Sensors.getCSDistanceMM());
        telemetry.addData("Intake Current", robotMotors.getIntakeCurrent());
        telemetry.addData("Robot State", robotState);
        telemetry.update();
    }

    @Override
    public void stop() {}
}
