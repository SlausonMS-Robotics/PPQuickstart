package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

/**
 * Simple static class to hold the robot's pose and other data between OpModes.
 */
public class PoseStorage {
    // The last known pose of the robot from an Autonomous OpMode
    public static Pose currentPose = new Pose(0, 0, 0);

    // The alliance color, which can be set in Autonomous and read in TeleOp
    public static String allianceColor = "blue";
}
