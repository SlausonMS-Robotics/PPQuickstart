package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.robot.PoseStorage;

public class limelight3A {

    public boolean LLOn = false;

    private Telemetry telemetry;

    // Hardware
    public Limelight3A limelight;

    // Results
    public LLResult result;
    private servos Servos;
    private Follower follower;
    private TurretAiming turretAimer;


    /**
     * Initializes the Limelight and headlight hardware.
     */
    public boolean init(HardwareMap hwMap, int pipeline, Telemetry telemetry, servos Servos, Follower follower) {

        try {
            limelight = hwMap.get(Limelight3A.class, "Limelight");
            this.telemetry = telemetry;
            this.Servos = Servos;
            this.follower = follower;
            this.turretAimer = turretAimer;


            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(pipeline);
            limelight.start();
            limelight.updateRobotOrientation(follower.getPose().getHeading());


            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    public void stopLL(){
        if (limelight != null) limelight.stop();
        LLOn = false;

    }
    public boolean pollLimelight() {

        if (limelight == null) return false;


        result = limelight.getLatestResult();
        return true;
    }

    public double getLLAvgDist(){
        if (result == null || !result.isValid()) return 0.0;
        return result.getBotposeAvgDist();
    }

    /**
     * Gets the robot's X position on the field from the Limelight's botpose.
     * @return The robot's X position in inches.
     */
    public double getLLFieldX() {
        if (result == null || !result.isValid()) return 0.0;
        Pose3D botpose = result.getBotpose();
        return 72 - (botpose.getPosition().x / .0254);
    }

    public double getLLFieldX_MT2() {
        if (result == null || !result.isValid()) return 0.0;
        Pose3D botpose = result.getBotpose_MT2();
        return 72 - (botpose.getPosition().x / .0254);
    }



    /**
     * Gets the robot's Y position on the field from the Limelight's botpose.
     * @return The robot's Y position in inches.
     */
    public double getLLFieldY() {
        if (result == null || !result.isValid()) return 0.0;
        Pose3D botpose = result.getBotpose();
        return 72 + (botpose.getPosition().y / .0254);
    }

    public double getLLFieldY_MT2() {
        limelight.updateRobotOrientation(follower.getPose().getHeading());
        if (result == null || !result.isValid()) return 0.0;
        Pose3D botpose = result.getBotpose_MT2();
        return 72 + (botpose.getPosition().y / .0254);
    }

    /**
     * Gets the robot's heading on the field from the Limelight's botpose.
     * @return The robot's heading (yaw) in degrees.
     */



}
