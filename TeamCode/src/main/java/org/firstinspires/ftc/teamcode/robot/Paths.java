package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class Paths {

    public PathChain Path_3rd_row_approach_blue;
    public PathChain Path_3rd_row_pickup_blue;
    public PathChain Path_3rd_row_shoot_back_blue;
    public PathChain Path_HP_approach_blue;
    public PathChain Path_HP_pickup_blue;
    public PathChain Path_HP_shoot_back_blue;
    public PathChain Path_2nd_row_approach_blue;
    public PathChain Path_2nd_row_pickup_blue;
    public PathChain Path_2nd_row_shoot_front_blue;
    public PathChain Path_1st_row_pickup_blue;

    public PathChain Path_1st_row_shoot_front_blue;
    public PathChain Path_finish_back_blue;

    public Paths(Follower follower) {
        Path_3rd_row_approach_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 8.000), new Pose(44.000, 36.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path_3rd_row_pickup_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 36.000), new Pose(20.000, 36.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path_3rd_row_shoot_back_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 36.000), new Pose(60.000, 16.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path_HP_approach_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 16.000), new Pose(8.000, 30.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        Path_HP_pickup_blue = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.000, 30.000), new Pose(8.000, 8.000)))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        Path_HP_shoot_back_blue = follower
                .pathBuilder()
                .addPath(new BezierLine(new Pose(8.000, 8.000), new Pose(60.000, 16.000)))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        Path_2nd_row_approach_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 16.000), new Pose(44.000, 57.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        Path_2nd_row_pickup_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(44.000, 57.000), new Pose(20.000, 57.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path_2nd_row_shoot_front_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.000, 57.000),
                                new Pose(36.000, 60.000),
                                new Pose(60.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path_1st_row_pickup_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(20.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path_1st_row_shoot_front_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(20.000, 84.000), new Pose(60.000, 84.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path_finish_back_blue = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(60.000, 84.000), new Pose(60.000, 44.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


    }
}


