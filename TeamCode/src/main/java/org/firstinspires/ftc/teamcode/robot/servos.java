package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

public class servos {

    // ---- Constants ----

    public static final double TURRET_MIN_POS = -.8;
    public static final double TURRET_MAX_POS = .8;
    

    // ---- Servos ----
    private ServoImplEx turretServo;     // servo0
    private ServoImplEx indexerServo;  // servo1

    /**
     * Initializes all servos.
     */
    public void init(HardwareMap hardwareMap) {
        
        turretServo = hardwareMap.get(ServoImplEx.class, "servohub0");
        indexerServo = hardwareMap.get(ServoImplEx.class, "servohub1");
    }

    public void setTurretServoPos(double pos) {
        if (turretServo != null) {
            turretServo.setPosition(Range.clip(pos, TURRET_MIN_POS, TURRET_MAX_POS));
        }
    }
    
    public void setIndexerServoPos(double pos) {
        if (indexerServo != null) indexerServo.setPosition(pos);
    }

}
