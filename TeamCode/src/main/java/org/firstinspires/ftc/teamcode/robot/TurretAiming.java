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

    // ---- Shooter State Members ----
    private double targetRPM = 0;
    private double targetBouncerPos = 0.5;

    // ---- Aiming Constants ----
    private static final int blueGoalX = 12;
    private static final int blueGoalY = 136;
    private static final int redGoalX = 136;
    private static final int redGoalY = 136;

    private static final double minTurretAngle = -110;
    private static final double maxTurretAngle = 110;

    // ---- Multi-Output Lookup Table Data ----
    // Independent Variable: Distance in meters
    private static final double[] LOOKUP_DISTANCES = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5};
    
    // Output 1: Flywheel RPM
    private static final double[] LOOKUP_RPMS = {2400, 2600, 2700, 2800, 3000, 3300};
    
    // Output 2: Bouncer Servo Position
    private static final double[] LOOKUP_BOUNCER_POS = {0.538, 0.55, 0.51, 0.47, 0.43, 0.4};

    public TurretAiming(Follower follower, limelight3A limelight, servos Servos, motors robotMotors, Telemetry telemetry, other_helpers helpers) {
        this.follower = follower;
        this.limelight = limelight;
        this.Servos = Servos;
        this.robotMotors = robotMotors;
        this.telemetry = telemetry;
        this.movingAverage = new other_helpers();
        movingAverage.initMovingAverage(5);
        llTimer.resetTimer();
    }

    /**
     * Sets the shooter parameters (RPM and Bouncer Position) based on distance.
     */
    public void setShooter(double distMeters) { //sets flywheel speed and bouncer position based on distance to goal
        if (distMeters > .05 && distMeters < 5) {
            // Clamp distance to the range of our lookup table
            double clampedDist = Math.max(LOOKUP_DISTANCES[0], Math.min(LOOKUP_DISTANCES[LOOKUP_DISTANCES.length - 1], distMeters));
            
            targetRPM = interpolate(clampedDist, LOOKUP_DISTANCES, LOOKUP_RPMS);
            targetBouncerPos = interpolate(clampedDist, LOOKUP_DISTANCES, LOOKUP_BOUNCER_POS);
        } else {
            targetRPM = 0;
            targetBouncerPos = 0.5;
        }
        
        robotMotors.setShooterVelocity(robotMotors.getShooterVelocityFromRPM(targetRPM));
        Servos.setBouncerServo(targetBouncerPos);
    }

    /**
     * Aims the turret and calculates RPM/Bouncer position using synchronized lookup tables.
     */
    public void updateOdomAiming(String myAllianceColor) {
        double distInches = distanceToGoalInches(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        double distMeters = .0254 * distInches;

        setShooter(distMeters);

        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        double targetFieldHeadingDeg = getFieldHeadingToGoal(follower.getPose().getX(), follower.getPose().getY(), myAllianceColor);
        
        // Servos.updateTurretWithPID(targetFieldHeadingDeg, robotHeadingDeg);

        if (telemetry != null) {
            telemetry.addData("AIMING MODE", "ODOMETRY (LOOKUP)");
            telemetry.addData("Dist (m)", "%.2f", distMeters);
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Target Bouncer", "%.3f", targetBouncerPos);
            
            if (limelight.result != null && limelight.result.getStaleness() < 500) {
                if (limelight.pollLimelight()) {
                    telemetry.addData("LL X", limelight.getLLFieldX());
                    telemetry.addData("LL Y", limelight.getLLFieldY());
                    telemetry.addData("LL Heading", getLLFieldHeadingDeg());
                }
            }
        }
        updateTelemetry();
    }

    /**
     * Generic linear interpolation helper.
     */
    public static double interpolate(double x, double[] xArr, double[] yArr) {
        if (x <= xArr[0]) return yArr[0];
        if (x >= xArr[xArr.length - 1]) return yArr[yArr.length - 1];
        
        for (int i = 0; i < xArr.length - 1; i++) {
            if (x <= xArr[i + 1]) {
                double x0 = xArr[i];
                double x1 = xArr[i + 1];
                double y0 = yArr[i];
                double y1 = yArr[i + 1];
                return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
            }
        }
        return yArr[yArr.length - 1];
    }

    public boolean updateLimelightAiming() {
        if (limelight == null || llTimer.getElapsedTime() < 10) return false;
        llTimer.resetTimer();
        limelight.pollLimelight();

        if (limelight.result.isValid()) {
            Servos.setLedColor(servos.LedColor.GREEN);
            double llGoalDist = limelight.getLLAvgDist();
            if (llGoalDist > .25 && llGoalDist < 4.5) {
                setShooter(llGoalDist);

                Servos.updateTurretWithPID(0, limelight.result.getTx());
            }
            return true;
        }
        Servos.setLedColor(servos.LedColor.RED);
        return false;
    }

    public void updatePoseFromLimelight() {
        if (limelight == null || !limelight.pollLimelight() || !limelight.result.isValid()) return;
        Pose newPose = getLLBotpose();
        if (newPose != null && newPose.getX() >= 0.0 && newPose.getY() >= 0.0) {
            follower.setPose(newPose);
        }
    }

    private void updateTelemetry() {
        if (telemetry != null) {
            telemetry.addData("Current Pose", follower.getPose());
            telemetry.update();
        }
    }

    private double distanceToBlueGoal(double currentX, double currentY) {
        return Math.sqrt(Math.pow(blueGoalX - currentX, 2) + Math.pow(blueGoalY - currentY, 2));
    }
    
    private double distanceToRedGoal(double currentX, double currentY) {
        return Math.sqrt(Math.pow(redGoalX - currentX, 2) + Math.pow(redGoalY - currentY, 2));
    }

    private double distanceToGoalInches(double currentX, double currentY, String allianceColor) {
        return "blue".equals(allianceColor) ? distanceToBlueGoal(currentX, currentY) : distanceToRedGoal(currentX, currentY);
    }

    public double getFieldHeadingToGoal(double currentX, double currentY, String allianceColor) {
        double targetX = "blue".equals(allianceColor) ? blueGoalX : redGoalX;
        double targetY = "blue".equals(allianceColor) ? blueGoalY : redGoalY;
        return Math.toDegrees(Math.atan2(targetY - currentY, targetX - currentX));
    }

    public double getLLFieldHeadingDeg() {
        if (limelight.result == null || !limelight.result.isValid()) return 0.0;
        double goalAngleDeg = getFieldHeadingToGoal(limelight.getLLFieldX(), limelight.getLLFieldY(), PoseStorage.allianceColor);
        return goalAngleDeg + Servos.getTurretRobotAngleDeg() + limelight.result.getTx();
    }

    public Pose getLLBotpose() {
        if (limelight.result == null || !limelight.result.isValid()) return null;
        return new Pose(limelight.getLLFieldX(), limelight.getLLFieldY(), Math.toRadians(getLLFieldHeadingDeg()));
    }
}
