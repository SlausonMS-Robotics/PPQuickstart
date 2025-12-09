package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretAiming {

    private final Follower follower;
    private final limelight3A limelight;
    private final servos Servos;
    private final motors robotMotors;
    private final Telemetry telemetry;
    private final other_helpers movingAverage;

    private final Timer llTimer = new Timer();
    private static final int ticks_per_rev = 28;

    // ---- Aiming Constants (moved from other_helpers) ----
    private static final int blueGoalX = 12;
    private static final int blueGoalY = 136;
    private static final int redGoalX = 136;
    private static final int redGoalY = 136;

    private static final double minTurretAngle = -110;
    private static final double maxTurretAngle = 110;

    // ---- Polynomial RPM Coefficients ----
    // These coefficients define the quadratic equation: RPM = A*x^2 + B*x + C
    // where x is the distance in meters. Derived from the new stepped data.
    private static final double POLY_A = 140;
    private static final double POLY_B = 100;
    private static final double POLY_C = 2300;
    private double previousHeadingError = 0;

    public TurretAiming(Follower follower, limelight3A limelight, servos Servos, motors robotMotors, Telemetry telemetry, other_helpers helpers) {
        this.follower = follower;
        this.limelight = limelight;
        this.Servos = Servos;
        this.robotMotors = robotMotors;
        this.telemetry = telemetry;
        this.movingAverage = helpers;
        movingAverage.initMovingAverage(5);
        llTimer.resetTimer();
    }

    /**
     * Aims the turret using odometry data as the default.
     */
    public void updateOdomAiming(String myAllianceColor) {
        // Set shooter speed based on odometry
        double targetRPM = 0;
        double odomDistanceMeters = .0254 * distanceToGoalInches(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        if (odomDistanceMeters > .05 && odomDistanceMeters < 5) {
            targetRPM = getRPMForDistancePolynomial(odomDistanceMeters);
            //targetRPM = getRPMForDistanceMeters(odomDistanceMeters);
            double speed = targetRPM * ticks_per_rev / 60;
            robotMotors.setShooterVelocity(speed);
        }

        // Aim with odometry
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        double targetFieldHeadingDeg = getFieldHeadingToGoal(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        
        //Servos.updateTurretWithPID(targetFieldHeadingDeg, robotHeadingDeg);

        
        if (telemetry != null) {
            telemetry.addData("AIMING MODE", "ODOMETRY (DEFAULT)");
            telemetry.addData("Robot Heading", robotHeadingDeg);
            telemetry.addData("Target Heading", targetFieldHeadingDeg);
            telemetry.addData("Distance to Goal (m)",odomDistanceMeters);
            telemetry.addData("Target RPM",targetRPM);
            telemetry.addData("Motor1 RPM", robotMotors.getShooterMotor1RPM());
            telemetry.addData("Motor0 RPM", robotMotors.getShooterMotor0RPM());
            if (limelight.result.getStaleness() >= 500 ) {

                if (limelight.pollLimelight()) {
                    telemetry.addData("Limelight X", limelight.getLLFieldX());
                    telemetry.addData("Limelight Y", limelight.getLLFieldY());
                    telemetry.addData("Limelight Heading", getLLFieldHeadingDeg());
                }
            }
        }
        

    }
    
    /**
     * Explicitly polls the Limelight and attempts to aim.
     * @return true if a valid target was found and used for aiming, false otherwise.
     */
    public boolean updateLimelightAiming() {
        if (limelight == null || llTimer.getElapsedTime() < 10) {
            return false; 
        }
        llTimer.resetTimer();

        if (limelight.pollLimelight() && limelight.result.isValid()) {
            Servos.setLedColor(servos.LedColor.GREEN);

            double llGoalHeadingError = limelight.result.getTx();
            double llGoalDist = limelight.getLLAvgDist();

            if (llGoalDist > 0 && llGoalDist < 5) {
                double targetRPM = getRPMForDistancePolynomial(llGoalDist);
                //double targetRPM = getRPMForDistanceMeters(llGoalDist);
                robotMotors.setShooterVelocity(targetRPM);
            }




            Servos.updateTurretWithPID(0, llGoalHeadingError);

            if (telemetry != null) {
                telemetry.addData("AIMING MODE", "LIMELIGHT");
            }

            return true; // Success
        }
        
        Servos.setLedColor(servos.LedColor.RED);
        return false;
    }

    public boolean llAim(boolean clear) {
        if (limelight == null || llTimer.getElapsedTime() < 20) {
            return false;
        }
        llTimer.resetTimer();
        if (clear) movingAverage.clearReadings();
        if (limelight.pollLimelight()) {
            if(!limelight.result.isValid()) return false;
            double llGoalHeadingError = limelight.result.getTx();
            if (Math.abs(llGoalHeadingError) <= .1) return false;
            if (limelight.getLLAvgDist() <= .2 || limelight.getLLAvgDist() >= 4) return false;
            double avgHeadingError = movingAverage.updateAndGetAverage(llGoalHeadingError);
            if (Math.abs(avgHeadingError) < 1 )
                Servos.setLedColor(servos.LedColor.YELLOW);
            else if (Math.abs(avgHeadingError) < .5)
                Servos.setLedColor(servos.LedColor.GREEN);
            else Servos.setLedColor(servos.LedColor.RED);
            Servos.updateTurretWithPID(0, avgHeadingError);

            if (telemetry != null) {
                telemetry.addData("AIMING MODE", "LIMELIGHT");
            }
            return true;
        }
        else return false;


    }

    /**
     * Updates the follower's pose with the latest data from the Limelight.
     * This should be called periodically to correct for odometry drift.
     */
    public void updatePoseFromLimelight() {
        if (limelight == null || !limelight.pollLimelight() || !limelight.result.isValid()) {
            return; // Do nothing if we have no valid data
        }

        double x = limelight.getLLFieldX();
        double y = limelight.getLLFieldY();

        // Only update if the Limelight data is reasonable (e.g., not 0,0,0)
        if (x >= 0.0 && y >= 0.0) {
            Pose newPose = getLLBotpose();
            follower.setPose(newPose);
            telemetry.addData("LL Bot Pose", newPose);
        }
    }


    // ---- Aiming Logic ----

    public static double getRPMForDistancePolynomial(double rangeMeters) {
        return POLY_A * Math.pow(rangeMeters, 2) + POLY_B * rangeMeters + POLY_C;
    }

    public static double getRPMForDistanceMeters(double rangeMeters) {
        if (rangeMeters <= 1.0) return 2500;
        if (rangeMeters <= 1.5) return 2700;
        if (rangeMeters <= 2.0) return 3050;
        if (rangeMeters <= 2.5) return 3400;
        if (rangeMeters <= 3.0) return 3900;
        if (rangeMeters <= 3.5) return 4250;
        return 4500;
    }
   
    private double distanceToBlueGoal(double currentX, double currentY) {
        double deltaX = blueGoalX - currentX;
        double deltaY = blueGoalY - currentY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }
    
    private double distanceToRedGoal(double currentX, double currentY) {
        double deltaX = redGoalX - currentX;
        double deltaY = redGoalY - currentY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private double getFieldHeadingToBlueGoal(double currentX, double currentY) {
        return Math.toDegrees(Math.atan2(blueGoalY - currentY, blueGoalX - currentX));
    }

    private double getFieldHeadingToRedGoal(double currentX, double currentY) {
        return Math.toDegrees(Math.atan2(redGoalY - currentY, redGoalX - currentX));
    }
    
    private double distanceToGoalInches(double currentX, double currentY, String allianceColor) {
        if ("blue".equals(allianceColor)) {
            return distanceToBlueGoal(currentX, currentY);
        } else {
            return distanceToRedGoal(currentX, currentY);
        }
    }

    public double getFieldHeadingToGoal(double currentX, double currentY, String allianceColor) {
        if ("blue".equals(allianceColor)) {
            return getFieldHeadingToBlueGoal(currentX, currentY);
        } else {
            return getFieldHeadingToRedGoal(currentX, currentY);
        }
    }

    public double getLLFieldHeadingDeg() {
        double llAngleDeg = 0;
        if (limelight.result != null && limelight.result.isValid()) {

            llAngleDeg = limelight.result.getTx(); // positive is to the right
        }
        else return 0.0;
        double servoAngleDeg = Servos.getTurretRobotAngleDeg(); //0 is straight ahead, positive is to the right
        double goalAngleDeg = getFieldHeadingToGoal(limelight.getLLFieldX(), limelight.getLLFieldY(), PoseStorage.allianceColor); //calculates where the goal is in terms of field angle (0 is to the right, 180 is left)



        return goalAngleDeg + servoAngleDeg + llAngleDeg; //example: servo is at 90deg (straight left relative to robot heading), LL says -5, goal heading is 135 then robot heading is 40
    }

    public Pose getLLBotpose() {
        if (limelight.result == null || !limelight.result.isValid()) return null;
        Pose botPose = new Pose(limelight.getLLFieldX(), limelight.getLLFieldY(), getLLFieldHeadingDeg());
        return botPose;
    }
}
