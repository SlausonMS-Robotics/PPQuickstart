package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

import com.qualcomm.hardware.limelightvision.LLResult;

public class SimpleTurretAimer {
    private final SimplifiedRobot robot;
    
    // Constants from original servos.java
    private static final double GEAR_RATIO = 86.0 / 42.0;
    private static final double SERVO_DEGREES_RANGE = 355.0;
    private static final double TURRET_CENTER_POS = 0.5;
    private static final double SERVO_UNITS_PER_DEGREE = (GEAR_RATIO / SERVO_DEGREES_RANGE);

    // Lookup table from original TurretAiming.java
    private static final double[] LOOKUP_DISTANCES = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    private static final double[] LOOKUP_RPMS = {2225, 2575, 2725, 3175, 3425, 3575};
    private static final double[] LOOKUP_BOUNCER_POS = {0.58, 0.55, 0.51, 0.48, 0.43, 0.4};

    public SimpleTurretAimer(SimplifiedRobot robot) {
        this.robot = robot;
    }

    public void setShooterByDistance(double distMeters) {
        double clampedDist = Math.max(LOOKUP_DISTANCES[0], Math.min(LOOKUP_DISTANCES[LOOKUP_DISTANCES.length - 1], distMeters));
        double targetRPM = interpolate(clampedDist, LOOKUP_DISTANCES, LOOKUP_RPMS);
        double targetBouncerPos = interpolate(clampedDist, LOOKUP_DISTANCES, LOOKUP_BOUNCER_POS);
        
        // Convert RPM to Ticks/Sec (assuming 28 ticks per rev, typical for some motors)
        double velocity = targetRPM * 28.0 / 60.0;
        robot.motors.setShooterVelocity(velocity);
        robot.servos.bouncerServo.setPosition(targetBouncerPos);
    }

    public boolean updateLimelightAiming() {
        LLResult result = robot.limelight.getResult();
        if (result != null && result.isValid()) {
            double errorDegrees = result.getTx();
            incrementTurret(errorDegrees);
            
            if (Math.abs(errorDegrees) < 1.0) {
                robot.servos.setLedColor(SimpleServos.LedColor.GREEN);
                return true;
            } else {
                robot.servos.setLedColor(SimpleServos.LedColor.YELLOW);
            }
        } else {
            robot.servos.setLedColor(SimpleServos.LedColor.RED);
        }
        return false;
    }

    private void incrementTurret(double degInc) {
        double currentPos = robot.servos.turretServo.getPosition();
        double newPos = currentPos - (degInc * SERVO_UNITS_PER_DEGREE); // Negative because Tx is usually opposite to rotation needed
        robot.servos.turretServo.setPosition(newPos);
    }

    private double interpolate(double x, double[] xArr, double[] yArr) {
        if (x <= xArr[0]) return yArr[0];
        if (x >= xArr[xArr.length - 1]) return yArr[yArr.length - 1];
        for (int i = 0; i < xArr.length - 1; i++) {
            if (x <= xArr[i + 1]) {
                return yArr[i] + (x - xArr[i]) * (yArr[i + 1] - yArr[i]) / (xArr[i + 1] - xArr[i]);
            }
        }
        return yArr[yArr.length - 1];
    }
}
