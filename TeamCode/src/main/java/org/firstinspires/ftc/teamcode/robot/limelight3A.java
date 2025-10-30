package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class limelight3A {

    public boolean LLOn = false;

    private Telemetry telemetry;

    // Hardware
    private Limelight3A limelight;
    // Results
    public LLResult result;

    /**
     * Initializes the Limelight and headlight hardware.
     */
    public boolean init(HardwareMap hwMap, int pipeline, Telemetry telemetry) {
        try {
            limelight = hwMap.get(Limelight3A.class, "Limelight");
            this.telemetry = telemetry;

            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(pipeline);


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
        return result.getBotposeAvgDist();
    }


}
