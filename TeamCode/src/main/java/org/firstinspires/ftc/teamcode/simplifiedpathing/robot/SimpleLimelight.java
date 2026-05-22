package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimpleLimelight {
    public Limelight3A limelight;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public LLResult getResult() {
        return limelight.getLatestResult();
    }
}
