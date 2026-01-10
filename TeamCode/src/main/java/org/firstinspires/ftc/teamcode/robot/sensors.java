package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class sensors {

    private DistanceSensor DISTSensor;
    //private AnalogInput analogServoSensor;

    //private IMU imu;


    /**
     * Initializes the sensors.
     */
    public void init(HardwareMap hardwareMap) {
        DISTSensor = hardwareMap.get(DistanceSensor.class, "ehi2c1");
        //analogServoSensor = hardwareMap.get(AnalogInput.class, "analog0");
        //imu = hardwareMap.get(IMU.class, "imu");
    }


    public double getCSDistanceMM() {
        if (DISTSensor == null) return 1000;
        else {
            return DISTSensor.getDistance(DistanceUnit.MM);
        }
    }
}
