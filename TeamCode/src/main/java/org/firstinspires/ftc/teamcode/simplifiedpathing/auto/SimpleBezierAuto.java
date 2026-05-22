package org.firstinspires.ftc.teamcode.simplifiedpathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimpleBezier;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

@Autonomous(name = "Simple Bezier Example", group = "Simplified")
public class SimpleBezierAuto extends LinearOpMode {
    SimplifiedRobot robot = new SimplifiedRobot();
    SimplifiedFollower follower;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new SimplifiedFollower(robot);

        // Define a curve: Start (0,0), Control (24, 0), End (24, 24)
        SimpleBezier curve = new SimpleBezier(
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 90)
        );

        waitForStart();

        follower.resetCurve();
        while (opModeIsActive()) {
            robot.sensors.pinpoint.update();
            Pose2D currentPose = robot.sensors.pinpoint.getPosition();

            boolean finished = follower.followCurve(currentPose, curve);
            
            if (finished) break;
            
            telemetry.addData("Status", "Following Curve");
            telemetry.update();
        }

        follower.stop();
    }
}
