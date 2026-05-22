package org.firstinspires.ftc.teamcode.simplifiedpathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * A much simpler alternative to complex pathing libraries.
 * It uses basic Proportional control to move to a target Pose2D.
 */
public class SimplifiedFollower {
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    
    // Simple Proportional gains - these will need tuning for your specific robot!
    public static double XY_GAIN = 0.05;
    public static double HEADING_GAIN = 0.03;
    public static double MAX_POWER = 0.8;

    public SimplifiedFollower(HardwareMap hardwareMap) {
        // Change these strings to match your configuration names
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // Most mecanum robots need one side reversed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        
        // Ensure motors stop when power is zero
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Calculates and applies motor powers to move toward a target pose.
     * @param current The robot's current position (e.g., from an OTOS, Pinpoint, or Odometry)
     * @param target The desired position
     * @return true if the robot has reached the target (within a small threshold)
     */
    public boolean update(Pose2D current, Pose2D target) {
        double xError = target.getX(DistanceUnit.INCH) - current.getX(DistanceUnit.INCH);
        double yError = target.getY(DistanceUnit.INCH) - current.getY(DistanceUnit.INCH);
        double headingError = AngleUnit.normalizeDegrees(target.getHeading(AngleUnit.DEGREES) - current.getHeading(AngleUnit.DEGREES));

        // Basic P-control for movement
        double xPower = xError * XY_GAIN;
        double yPower = yError * XY_GAIN;
        double turnPower = headingError * HEADING_GAIN;

        // Rotate movement vector to be field-centric so the robot moves relative to the field
        double robotHeading = current.getHeading(AngleUnit.RADIANS);
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        
        double fieldX = xPower * cos - yPower * sin;
        double fieldY = xPower * sin + yPower * cos;

        drive(fieldX, fieldY, turnPower);

        // Return true if within 1 inch and 2 degrees of target
        return Math.abs(xError) < 1.0 && Math.abs(yError) < 1.0 && Math.abs(headingError) < 2.0;
    }

    /**
     * Classic mecanum drive calculation
     */
    public void drive(double x, double y, double turn) {
        double lf = x + y + turn;
        double rf = x - y - turn;
        double lr = x - y + turn;
        double rr = x + y - turn;

        // Normalize powers if any exceed 1.0
        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr)))));
        
        leftFront.setPower(Range.clip(lf / max, -MAX_POWER, MAX_POWER));
        rightFront.setPower(Range.clip(rf / max, -MAX_POWER, MAX_POWER));
        leftRear.setPower(Range.clip(lr / max, -MAX_POWER, MAX_POWER));
        rightRear.setPower(Range.clip(rr / max, -MAX_POWER, MAX_POWER));
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
