package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.servos;

@Autonomous(name = "Example Sample Auto", group = "Examples")
public class ExampleSampleAuto extends OpMode {
    private Follower follower;

    servos robotservo = new servos();
    limelight3A limelight = new limelight3A();
    private Timer pathTimer;
    private int pathState;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose scorePose = new Pose(20, 0, Math.toRadians(45));
    private final Pose pickup1Pose = new Pose(20, 20, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(0, 20, Math.toRadians(45));
    private final Pose pickup3Pose = new Pose(0, 0, Math.toRadians(90));
    private final Pose parkPose = new Pose(20, 0, Math.toRadians(0));

    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();


    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /*
                 * Following the first path and setting holdEnd to true.
                 * This allows the robot to hold its endpoint if the path being followed is a PathChain.
                 * Without this,
                 */
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:
                /*
                 * There are many ways to check if your robot has reached its target pose.
                 * We will provide examples of all of them in this autonomous program.
                 * follower.atParametricEnd(): Checks if the t-value (path completion value) is past your pathEndTValueConstraint.
                 * follower.atPose(): Checks if the robot is at a certain pose (x, y, and heading) given error constraints for each.
                 * follower.getCurrentTValue(): Gets the t-value (path completion value).
                 * follower.isBusy(): Checks the robot's position and waits until the robot position is 1 inch away from its target.
                 */

                /* Checks the robot's position and waits until the robot position is 1 inch away from its target. */
                if (!follower.isBusy()) {
                    /*
                     * Open outtake claw
                     * This can be done with a method like "servo.setPosition(OUTTAKE_CLAW_OPEN)"
                     */

                    setPathState(2);
                }
                break;
            case 2:
                /*
                 * Checks to see if a second has passed before continuing with the next path.
                 * We will not include these cases in the rest of the auto, but it is a way to perform actions after running paths.
                 */
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(grabPickup1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* Checks the robot's position and waits until the robot position is 1 inch away from its target. */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(4);
                }
                break;
            case 4:
                // Checks if the t-value (path completion value) is past your pathEndTValueConstraint.
                if (follower.atParametricEnd()) {

                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                        follower.followPath(grabPickup2, true);
                        setPathState(5);
                    }
                }
                break;
            case 5:
                // Checks if the t-value (path completion value) is past your pathEndTValueConstraint.
                if (follower.atParametricEnd()) {
                    /* Grab Sample */
                    follower.followPath(scorePickup2,true);
                    setPathState(6);
                }
                break;
            case 6:
                // Checks if the robot is at the scorePose given a x-tolerance of 1 inch, a y-tolerance of 1 inch, and a heading tolerance of 5 degrees.
                if (follower.atPose(scorePose, 2, 2, Math.toRadians(15))) {
                    /* Score Sample */
                    follower.followPath(grabPickup3,true);
                    setPathState(7);
                }
                break;
            case 7:
                // Checks if the robot is at the grabPickup3 pose given a x-tolerance of 1 inch, a y-tolerance of 1 inch, and a heading tolerance of 5 degrees.
                if (follower.atPose(pickup3Pose, 2, 2, Math.toRadians(15))) {
                    /* Grab Sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(8);
                }
                break;
            case 8:
                // Checks if the current path's t-value (path completion value) is greater than 0.98 (98% done)
                if (follower.getCurrentTValue() > 0.98) {
                    /* Score Sample */

                    // A demonstration of what will happen if you choose to not holdEnd. The robot should bounce off the submersible edge.
                    follower.followPath(park,false);
                    setPathState(9);
                }
                break;
            case 9:
                // Checks if the current path's t-value (path completion value) is greater than 0.95 (95% done)
                if (follower.getCurrentTValue() > 0.94) {
                    /* Level 1 Ascent */
                    // Set the state to a case we won't use or define, so it just stops running a new path
                    setPathState(-1);
                }
                break;
        }
    }
    @Override
    public void loop() {
        // Loop robot movement and odometry values
        follower.update();
        // Loop the finite-state machine
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        robotservo.init(hardwareMap,telemetry);
        //limelight.init(hardwareMap,5, telemetry, S);
        buildPaths();
    }

    /** This method is called once at the start of the OpMode. It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        setPathState(0);
    }

}