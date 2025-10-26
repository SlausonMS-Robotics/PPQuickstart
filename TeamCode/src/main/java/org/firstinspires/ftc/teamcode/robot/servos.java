package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class servos {

    // ---- Constants ----

    public static final double SLIDE_FULL_EXTENSION_POS = 0.65;
    public static final double SLIDE_FULL_EXTENSION_DEG = 37.5;
    public static final double SLIDE_FULL_RETRACTION_POS = 0.5;
    public static final double SLIDE_FULL_RETRACTION_DEG = 80;
    public static final double WRIST_FULL_EXTENSION_POS = 0.4;
    public static final double WRIST_FULL_RETRACTION_POS = .85;

    public static final double SLIDE_SERVO_MAX_DEG = 170;

    public static final double SLIDE_DRIVE_ARM_LENGTH = 11.25;
    public static final double SLIDE_FULL_RETRACTION_IN = 4.0;
    public static final double SLIDE_FULL_EXTENSION_IN = 14.0;

    public static final double GRIPPER_OPEN = 0.33;
    public static final double GRIPPER_CLOSE = 0.48;

    // ---- Servos ----
    private ServoImplEx slideServo;    // servo5
    private ServoImplEx headlight;     // servo0
    private ServoImplEx wristServo;    // servo4
    private ServoImplEx gripperServo;  // servo3

    /**
     * Initializes all servos.
     */
    public void init(HardwareMap hardwareMap) {
        slideServo = hardwareMap.get(ServoImplEx.class, "servohub0");
        headlight = hardwareMap.get(ServoImplEx.class, "Servo0");
        wristServo = hardwareMap.get(ServoImplEx.class, "servohub2");
        gripperServo = hardwareMap.get(ServoImplEx.class, "servohub1");
    }

    // ---- Slide Control ----

    /**
     * Converts linear extension (inches) to servo position
     */
    public double setSlideInches(double inches) {
        inches = SLIDE_FULL_RETRACTION_IN + Math.max(0, Math.min(SLIDE_FULL_EXTENSION_IN, inches));
        double degrees =  Math.toDegrees(Math.acos(inches/(2 * SLIDE_DRIVE_ARM_LENGTH)));
        double degreeOffset = 1;
        double servoPos = (degrees + degreeOffset) / SLIDE_SERVO_MAX_DEG; //set physical servo min position to horizontal
        //double servoPos = SLIDE_FULL_EXTENSION_DEG + ((degrees-SLIDE_FULL_RETRACTION_DEG)*(SLIDE_FULL_EXTENSION_POS-SLIDE_FULL_EXTENSION_DEG))/SLIDE_FULL_EXTENSION_DEG-SLIDE_FULL_RETRACTION_DEG;
        if (slideServo != null) slideServo.setPosition(servoPos);
        return servoPos;
    }

    public void setSlideServoPos(double pos) {
        if (slideServo != null) slideServo.setPosition(pos);
    }

    public boolean slideFullExtend() {
        setSlideServoPos(SLIDE_FULL_EXTENSION_POS);
        return true;
    }

    public boolean slideFullRetract() {
        setSlideServoPos(SLIDE_FULL_RETRACTION_POS);
        return false;
    }

    public void slideServoOff() {
        if (slideServo != null) slideServo.setPwmDisable();
    }

    public void slideServoOn() {
        if (slideServo != null) slideServo.setPwmEnable();
    }

    // ---- Gripper Control ----

    public void setGripperServo(double pos) {
        if (gripperServo != null) gripperServo.setPosition(pos);
    }

    public boolean openGripper() {
        setGripperServo(GRIPPER_OPEN);
        return true;
    }

    public boolean closeGripper() {
        setGripperServo(GRIPPER_CLOSE);
        return false;
    }

    // ---- Wrist Control ----

    public void setWristServo(double pos) {
        if (wristServo != null) wristServo.setPosition(pos);
    }

    public boolean wristFullExtend() {
        setWristServo(WRIST_FULL_EXTENSION_POS);
        return true;
    }

    public boolean wristFullRetract() {
        setWristServo(WRIST_FULL_RETRACTION_POS);
        return false;
    }

    // ---- Headlight ----

    public boolean headlightOn() {
        if (headlight != null) headlight.setPosition(1);
        return true;
    }

    public boolean headlightOff() {
        if (headlight != null) headlight.setPosition(0);
        return false;
    }
}
