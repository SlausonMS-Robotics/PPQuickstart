package org.firstinspires.ftc.teamcode.simplifiedpathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

/**
 * An example Autonomous OpMode using the SimplifiedFollower and SimplifiedRobot.
 */
@Autonomous(name = "Simple Robot Pathing", group = "Simplified")
public class SimplePathOpMode extends LinearOpMode {
    SimplifiedRobot robot = new SimplifiedRobot();
    SimplifiedFollower follower;
    
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new SimplifiedFollower(robot);
        
        telemetry.addLine("Ready! Robot Initialized.");
        telemetry.update();

        waitForStart();

        // Step 1: Move to (24, 0)
        runToPose(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0));
        
        // Step 2: Move to (24, 24) and turn to 90 degrees
        runToPose(new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 90));

        // Step 3: Return home
        runToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

        follower.stop();
        telemetry.addLine("Path Complete");
        telemetry.update();
        sleep(2000);
    }

    private void runToPose(Pose2D target) {
        while (opModeIsActive()) {
            robot.sensors.pinpoint.update();
            Pose2D currentPose = robot.sensors.pinpoint.getPosition();
            
            boolean reached = follower.update(currentPose, target);
            
            telemetry.addData("Target", target.getX(DistanceUnit.INCH) + ", " + target.getY(DistanceUnit.INCH));
            telemetry.addData("Current", currentPose.getX(DistanceUnit.INCH) + ", " + currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();

            if (reached) break;
        }
    }
}
