package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Master class for the simplified robot hardware abstraction.
 */
public class SimplifiedRobot {
    public SimpleMotors motors = new SimpleMotors();
    public SimpleSensors sensors = new SimpleSensors();
    public SimpleServos servos = new SimpleServos();
    public SimpleLimelight limelight = new SimpleLimelight();

    public void init(HardwareMap hardwareMap) {
        motors.init(hardwareMap);
        sensors.init(hardwareMap);
        servos.init(hardwareMap);
        limelight.init(hardwareMap);
    }
}
