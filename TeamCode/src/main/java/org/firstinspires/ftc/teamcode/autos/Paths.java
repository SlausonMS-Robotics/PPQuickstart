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
        Red_Solo_Auto,
        // Add more auto path names here
        Blue_Solo_Auto
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

    public PathChain Path11;

    /**
     * Constructor that builds the selected set of path chains.
     * @param follower The Follower object from Pedro Pathing to build the paths with.
     * @param pathSelection The enum selecting which auto path to build.
     */
    public Paths(Follower follower, AutoPath pathSelection) {
        switch (pathSelection) {
            case Red_Solo_Auto:
                Path1 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(96.000, 8.000),

                                        new Pose(84.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))

                        .build();

                Path2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 84.000),

                                        new Pose(94.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(94.000, 84.000),

                                        new Pose(127.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(127.000, 84.000),

                                        new Pose(84.200, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(84.200, 84.000),
                                        new Pose(97.000, 60.000),
                                        new Pose(96.000, 58.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                        .build();

                Path6 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(96.000, 58.000),

                                        new Pose(134.000, 58.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path7 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(134.000, 58.000),
                                        new Pose(94.000, 60.000),
                                        new Pose(84.000, 83.900)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                        .build();

                Path8 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(84.000, 83.900),
                                        new Pose(94.000, 64.000),
                                        new Pose(96.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-1))

                        .build();

                Path9 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(96.000, 36.000),

                                        new Pose(134.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path10 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(134.000, 36.000),

                                        new Pose(84.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))

                        .build();

                Path11 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 84.000),

                                        new Pose(109.000, 70.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))

                        .build();
                break;

                case Blue_Solo_Auto:
                // Copy and paste your new path definitions from your external tool here
                    Path1 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(48.000, 8.000),

                                            new Pose(60.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))

                            .build();

                    Path2 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(60.000, 84.000),

                                            new Pose(50.000, 82.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                            .build();

                    Path3 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(50.000, 82.000),

                                            new Pose(21.000, 82.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                            .build();

                    Path4 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(21.000, 82.000),

                                            new Pose(59.800, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                            .build();

                    Path5 = follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(59.800, 84.000),
                                            new Pose(47.000, 60.000),
                                            new Pose(48.000, 58.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                            .build();

                    Path6 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(48.000, 58.000),

                                            new Pose(12.000, 58.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                            .build();

                    Path7 = follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(12.000, 58.000),
                                            new Pose(50.000, 60.000),
                                            new Pose(60.000, 83.900)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                            .build();

                    Path8 = follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(60.000, 83.900),
                                            new Pose(50.000, 64.000),
                                            new Pose(48.000, 36.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                            .build();

                    Path9 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(48.000, 36.000),

                                            new Pose(12.000, 36.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                            .build();

                    Path10 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(12.000, 36.000),

                                            new Pose(60.000, 84.000)
                                    )
                            ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))

                            .build();
                    break;
        }
    }

}
