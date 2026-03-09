package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.TurretAiming;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.other_helpers;
import org.firstinspires.ftc.teamcode.robot.servos;
import org.firstinspires.ftc.teamcode.robot.states;

@Autonomous(name = "Blue 12 Rear 1 Auto", group = "Autonomous")
@Configurable // Panels
public class Blue_12_Rear_1 extends OpMode {

    private String myAllianceColor = "blue";
    private other_helpers helpers;
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Robot hardware and logic classes
    private motors robotMotors;
    states robotStateController = new states();
    private int robotState = 0;
    private servos Servos;
    private limelight3A limelight;
    private TurretAiming turretAimer;

    private Timer pathTimer, actionTimer, opmodeTimer;

    private double timerCounter = 0;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware
        robotMotors = new motors();
        robotMotors.init(hardwareMap);
        Servos = new servos();
        Servos.init(hardwareMap, telemetry);
        limelight = new limelight3A();


        // Initialize path follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(48, 8, Math.toRadians(90)));
        limelight.init(hardwareMap, 0, telemetry, Servos, follower);

        // Initialize reusable aiming class
        //turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry);

        paths = new Paths(follower, Paths.AutoPath.AUTO_PATH2); // Build paths from the external Paths class
        turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry, helpers);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        actionTimer.resetTimer();
        opmodeTimer.resetTimer();
    }

    @Override
    public void loop() {
        //if (opmodeTimer.getElapsedTime() > 500) {delay a little to make sure robot is ready to shoot
        //robotMotors.shoot();
    //}

        follower.update(); // Update Pedro Pathing
        //turretAimer.update(myAllianceColor, false); // Update turret aim and shooter speed continuously
        try {
            autonomousPathUpdate(); // Update autonomous state machine
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0: //drive up to shooting position

                //if(pathTimer.getElapsedTime() > 2000) {
                    //robotMotors.stopTransfer();
                    //robotMotors.toggleIntake();
                    if (!follower.isBusy()){
                        turretAimer.setShooter(1);
                        follower.followPath(paths.Path1,false);
                        setPathState(1);
                        timerCounter = 0;
                     }

               // }
                break;
            case 1: //shoot + drive to grab ball set 1
                if (!follower.isBusy()) {
                    shoot();

                    if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                        follower.followPath(paths.Path2, false);
                        setPathState(2);
                        robotState = 1; //turn intake on
                    }
                    timerCounter++;
                }
                break;
            case 2: // grab first set of balls
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, .5,false);
                    setPathState(3);
                }
                break;
            case 3: //drive back to shoot
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, false);
                    setPathState(4);
                    timerCounter = 0;
                }
                break;
            case 4: //shoot + move to ball set 2
                if (!follower.isBusy()) {
                    shoot();
                    if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                        follower.followPath(paths.Path5, false);
                        setPathState(5);
                        robotState = 1;
                    }
                timerCounter++;
                }
                break;
            case 5: //pick up ball set 2
                if (!follower.isBusy()) {

                    follower.followPath(paths.Path6, .5,false);
                    setPathState(6);
                }
                break;
            case 6: //drive to shoot pos
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, false);
                    setPathState(7);
                }
                break;
            case 7: //shoot ball + move to end
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, false);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, false);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }

        robotStateController.setRobotState(robotState, Servos, robotMotors);
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void shoot(){
        if (timerCounter == 0) {
            pathTimer.resetTimer();
        }
        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
            robotState = 2;
        }
    }

    public void setActionState(int aState) {
        pathState = aState;
        actionTimer.resetTimer();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose(); // Save current pose to PoseStorage
        PoseStorage.allianceColor = myAllianceColor; //Save alliance color to PoseStorage
    }

}
