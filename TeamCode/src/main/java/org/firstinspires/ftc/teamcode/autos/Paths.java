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
        RED_12_REAR_1,
        // Add more auto path names here
        AUTO_PATH2
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
                Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(42.000, 36.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 36.000), new Pose(20.000, 36.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(20.000, 36.000), new Pose(56.000, 14.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 14.000), new Pose(42.000, 60.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 60.000), new Pose(20.000, 60.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(20.000, 60.000), new Pose(58.000, 78.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(58.000, 78.000), new Pose(42.000, 84.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 84.000), new Pose(20.000, 84.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(20.000, 84.000), new Pose(54.000, 84.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(54.000, 84.000), new Pose(22.000, 72.000))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270)).build();
                break;

            case RED_12_REAR_1:
                // Path definitions for the Red Alliance, Rear Position 1
                // NOTE: These are currently copies of the blue path and need to be updated.
                Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(42.000, 36.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path2 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 36.000), new Pose(20.000, 36.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path3 = follower.pathBuilder().addPath(new BezierLine(new Pose(20.000, 36.000), new Pose(56.000, 14.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path4 = follower.pathBuilder().addPath(new BezierLine(new Pose(56.000, 14.000), new Pose(42.000, 60.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path5 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 60.000), new Pose(20.000, 60.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path6 = follower.pathBuilder().addPath(new BezierLine(new Pose(20.000, 60.000), new Pose(58.000, 78.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path7 = follower.pathBuilder().addPath(new BezierLine(new Pose(58.000, 78.000), new Pose(42.000, 84.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path8 = follower.pathBuilder().addPath(new BezierLine(new Pose(42.000, 84.000), new Pose(20.000, 84.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path9 = follower.pathBuilder().addPath(new BezierLine(new Pose(20.000, 84.000), new Pose(54.000, 84.000))).setConstantHeadingInterpolation(Math.toRadians(180)).build();
                Path10 = follower.pathBuilder().addPath(new BezierLine(new Pose(54.000, 84.000), new Pose(22.000, 72.000))).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270)).build();
                break;

                case AUTO_PATH2:
                // Copy and paste your new path definitions from your external tool here
                Path1 = follower.pathBuilder().addPath(new BezierLine(new Pose(48.000, 8.000), new Pose(48.000, 96.000))).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135)).build();
                Path2 = follower.pathBuilder().addPath(new BezierCurve( new Pose(48.000, 96.000), new Pose(45.0, 83), new Pose(42.000, 84.500) ) ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)) .build();
                Path3 = follower.pathBuilder().addPath( new BezierLine( new Pose(42.000, 84.500), new Pose(16.000, 84.500) ) ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)) .build();
                Path4 = follower.pathBuilder().addPath( new BezierLine( new Pose(16.000, 84.500), new Pose(48.000, 96.000) ) ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)) .build();
                Path5 = follower.pathBuilder().addPath( new BezierCurve( new Pose(48.000, 96.000), new Pose(45.000, 60.000), new Pose(42.000, 60.000) ) ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180)) .build();
                Path6 = follower.pathBuilder().addPath( new BezierLine( new Pose(42.000, 60.000), new Pose(16.000, 60.000) ) ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)) .build();
                Path7 = follower.pathBuilder().addPath( new BezierLine( new Pose(16.000, 60.000), new Pose(48.000, 96.000) ) ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135)) .build();
                break;
        }
    }
}
