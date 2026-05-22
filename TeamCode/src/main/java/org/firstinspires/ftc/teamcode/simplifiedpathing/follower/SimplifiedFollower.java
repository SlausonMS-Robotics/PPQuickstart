package org.firstinspires.ftc.teamcode.simplifiedpathing.follower;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

/**
 * A simpler pathing library that uses the SimplifiedRobot abstraction.
 */
public class SimplifiedFollower {
    private final SimplifiedRobot robot;
    
    public static double XY_GAIN = 0.05;
    public static double HEADING_GAIN = 0.03;
    public static double MAX_POWER = 0.8;

    // Curve following parameters
    private double currentT = 0;
    public static double T_STEP = 0.01; // How fast to move along the curve

    public SimplifiedFollower(SimplifiedRobot robot) {
        this.robot = robot;
    }

    /**
     * Standard point-to-point update.
     */
    public boolean update(Pose2D current, Pose2D target) {
        double xError = target.getX(DistanceUnit.INCH) - current.getX(DistanceUnit.INCH);
        double yError = target.getY(DistanceUnit.INCH) - current.getY(DistanceUnit.INCH);
        double headingError = AngleUnit.normalizeDegrees(target.getHeading(AngleUnit.DEGREES) - current.getHeading(AngleUnit.DEGREES));

        double xPower = xError * XY_GAIN;
        double yPower = yError * XY_GAIN;
        double turnPower = headingError * HEADING_GAIN;

        double robotHeading = current.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        
        double fieldX = xPower * cos - yPower * sin;
        double fieldY = xPower * sin + yPower * cos;

        drive(fieldX, fieldY, turnPower);

        return Math.abs(xError) < 1.0 && Math.abs(yError) < 1.0 && Math.abs(headingError) < 2.0;
    }

    /**
     * Curve following update. Moves 't' along the curve based on distance to the current target point.
     */
    public boolean followCurve(Pose2D current, SimpleBezier curve) {
        Pose2D target = curve.getPoint(currentT);
        
        // Calculate distance to current target point on curve
        double dist = Math.hypot(
            target.getX(DistanceUnit.INCH) - current.getX(DistanceUnit.INCH),
            target.getY(DistanceUnit.INCH) - current.getY(DistanceUnit.INCH)
        );

        // If we are close to the current point, move 't' forward
        if (dist < 4.0 && currentT < 1.0) {
            currentT += T_STEP;
        }

        update(current, target);

        // Return true when we reach the very end of the curve
        if (currentT >= 1.0) {
            return update(current, curve.getPoint(1.0));
        }
        return false;
    }

    public void resetCurve() {
        currentT = 0;
    }

    public void drive(double x, double y, double turn) {
        double lf = x + y + turn;
        double rf = x - y - turn;
        double lr = x - y + turn;
        double rr = x + y - turn;

        // Corrected normalization
        double max = 1.0;
        max = Math.max(max, Math.abs(lf));
        max = Math.max(max, Math.abs(rf));
        max = Math.max(max, Math.abs(lr));
        max = Math.max(max, Math.abs(rr));
        
        robot.motors.leftFront.setPower(Range.clip(lf / max, -MAX_POWER, MAX_POWER));
        robot.motors.rightFront.setPower(Range.clip(rf / max, -MAX_POWER, MAX_POWER));
        robot.motors.leftRear.setPower(Range.clip(lr / max, -MAX_POWER, MAX_POWER));
        robot.motors.rightRear.setPower(Range.clip(rr / max, -MAX_POWER, MAX_POWER));
    }

    /**
     * Drives the robot relative to the field.
     * @param x The forward/backward power relative to the field
     * @param y The left/right power relative to the field
     * @param turn The rotation power
     * @param currentHeading The robot's current heading (usually from pinpoint)
     */
    public void driveFieldCentric(double x, double y, double turn, double currentHeading) {
        double cos = Math.cos(-currentHeading);
        double sin = Math.sin(-currentHeading);

        double robotX = x * cos - y * sin;
        double robotY = x * sin + y * cos;

        drive(robotX, robotY, turn);
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
