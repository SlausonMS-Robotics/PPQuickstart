package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * This class holds various autonomous path definitions for the robot.
 * By centralizing path creation, paths can be easily reused across different OpModes.
 */
public class Paths {

    /**
     * Enum to select which autonomous path set to build.
     */
    public enum AutoPath {
        BLUE_12_REAR_1,
        RED_12_REAR_1
        // Add more auto path names here
    }

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;

    /**
     * Constructor that builds the selected set of path chains.
     * @param follower The Follower object from Pedro Pathing to build the paths with.
     * @param pathSelection The enum selecting which auto path to build.
     */
    public Paths(Follower follower, AutoPath pathSelection) {
        switch (pathSelection) {
            case BLUE_12_REAR_1:
                // Path definitions for the Blue Alliance, Rear Position 1


                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(44.000, 8.000), new Pose(50.000, 33.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(50, 33.000), new Pose(12.000, 33.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(12, 32.000), new Pose(46.000, 10.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                        .build();



        break;

            case RED_12_REAR_1:
                // Path definitions for the Red Alliance, Rear Position 1
                // NOTE: These are currently copies of the blue path and need to be updated.


                Path2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(68.000, 8.000), new Pose(65.000, 10))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                        .build();

                Path3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(65, 33.000), new Pose(134.000, 33.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build();

                Path4 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(134, 33.000), new Pose(68.000, 10.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                        .build();
                break;

            /*
            case NEW_AUTO_PATH:
                // Copy and paste your new path definitions from your external tool here
                Path1 = ...
                Path2 = ...
                // etc.
                break;
            */
        }
    }
}
