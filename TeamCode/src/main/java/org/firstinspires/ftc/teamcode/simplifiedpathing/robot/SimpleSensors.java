package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SimpleSensors {
    public GoBildaPinpointDriver pinpoint;
    public DistanceSensor intakeDist;

    public void init(HardwareMap hardwareMap) {
        // Localization
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setOffsets(-7.75, 2, DistanceUnit.INCH); 
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        // Intake Distance Sensor
        intakeDist = hardwareMap.get(DistanceSensor.class, "ehi2c1");
    }

    public double getIntakeDistanceMM() {
        if (intakeDist == null) return 1000;
        return intakeDist.getDistance(DistanceUnit.MM);
    }
}
