package org.firstinspires.ftc.teamcode.simplifiedpathing.robot;

import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * A simpler pathing library that uses the SimplifiedRobot abstraction.
 */
public class SimplifiedFollower {
    private final SimplifiedRobot robot;
    
    public static double XY_GAIN = 0.05;
    public static double HEADING_GAIN = 0.03;
    public static double MAX_POWER = 0.8;

    public SimplifiedFollower(SimplifiedRobot robot) {
        this.robot = robot;
    }

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

    public void stop() {
        drive(0, 0, 0);
    }
}
