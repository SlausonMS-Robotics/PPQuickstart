package org.firstinspires.ftc.teamcode.simplifiedpathing.follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

/**
 * A simple Bezier curve calculator.
 */
public class SimpleBezier {
    private final List<Pose2D> controlPoints = new ArrayList<>();

    public SimpleBezier(Pose2D p0, Pose2D p1, Pose2D p2) {
        controlPoints.add(p0);
        controlPoints.add(p1);
        controlPoints.add(p2);
    }

    public SimpleBezier(Pose2D p0, Pose2D p1, Pose2D p2, Pose2D p3) {
        controlPoints.add(p0);
        controlPoints.add(p1);
        controlPoints.add(p2);
        controlPoints.add(p3);
    }

    /**
     * Calculates the pose at point 't' along the curve (0.0 to 1.0).
     */
    public Pose2D getPoint(double t) {
        t = Math.max(0, Math.min(1, t));
        if (controlPoints.size() == 3) {
            return calculateQuadratic(t);
        } else {
            return calculateCubic(t);
        }
    }

    private Pose2D calculateQuadratic(double t) {
        Pose2D p0 = controlPoints.get(0);
        Pose2D p1 = controlPoints.get(1);
        Pose2D p2 = controlPoints.get(2);

        double x = (1 - t) * (1 - t) * p0.getX(DistanceUnit.INCH) + 
                   2 * (1 - t) * t * p1.getX(DistanceUnit.INCH) + 
                   t * t * p2.getX(DistanceUnit.INCH);
                   
        double y = (1 - t) * (1 - t) * p0.getY(DistanceUnit.INCH) + 
                   2 * (1 - t) * t * p1.getY(DistanceUnit.INCH) + 
                   t * t * p2.getY(DistanceUnit.INCH);

        // Interpolate heading linearly
        double heading = p0.getHeading(AngleUnit.DEGREES) + 
                         t * AngleUnit.normalizeDegrees(p2.getHeading(AngleUnit.DEGREES) - p0.getHeading(AngleUnit.DEGREES));

        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }

    private Pose2D calculateCubic(double t) {
        Pose2D p0 = controlPoints.get(0);
        Pose2D p1 = controlPoints.get(1);
        Pose2D p2 = controlPoints.get(2);
        Pose2D p3 = controlPoints.get(3);

        double invT = 1 - t;
        double x = invT * invT * invT * p0.getX(DistanceUnit.INCH) +
                   3 * invT * invT * t * p1.getX(DistanceUnit.INCH) +
                   3 * invT * t * t * p2.getX(DistanceUnit.INCH) +
                   t * t * t * p3.getX(DistanceUnit.INCH);

        double y = invT * invT * invT * p0.getY(DistanceUnit.INCH) +
                   3 * invT * invT * t * p1.getY(DistanceUnit.INCH) +
                   3 * invT * t * t * p2.getY(DistanceUnit.INCH) +
                   t * t * t * p3.getY(DistanceUnit.INCH);

        double heading = p0.getHeading(AngleUnit.DEGREES) + 
                         t * AngleUnit.normalizeDegrees(p3.getHeading(AngleUnit.DEGREES) - p0.getHeading(AngleUnit.DEGREES));

        return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }
}
