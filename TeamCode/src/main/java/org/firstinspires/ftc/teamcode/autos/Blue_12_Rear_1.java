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
import org.firstinspires.ftc.teamcode.robot.servos;

@Autonomous(name = "Blue 12 Rear 1 Auto", group = "Autonomous")
@Configurable // Panels
public class Blue_12_Rear_1 extends OpMode {

    private String myAllianceColor = "blue";
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // Robot hardware and logic classes
    private motors robotMotors;
    private servos Servos;
    private limelight3A limelight;
    private TurretAiming turretAimer;

    private Timer pathTimer, actionTimer, opmodeTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initialize hardware
        robotMotors = new motors();
        robotMotors.init(hardwareMap);
        Servos = new servos();
        Servos.init(hardwareMap);
        limelight = new limelight3A();
        limelight.init(hardwareMap, 0, telemetry);

        // Initialize path follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        // Initialize reusable aiming class
        turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry);

        paths = new Paths(follower, Paths.AutoPath.BLUE_12_REAR_1); // Build paths from the external Paths class

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
        follower.update(); // Update Pedro Pathing
        turretAimer.update(myAllianceColor); // Update turret aim and shooter speed continuously
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (opmodeTimer.getElapsedTime() > 500) { //delay a little to make sure robot is ready to shootAction
                    shootAction();
                }
                if(pathTimer.getElapsedTime() > 1500){
                    intakeAction();
                    follower.followPath(paths.Path1);
                    setPathState(1);

                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path10, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void shootAction(){

        robotMotors.shoot();
        actionTimer.resetTimer();

    }

    public void intakeAction() {

        robotMotors.stopTransfer();
        robotMotors.toggleIntake();
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
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
