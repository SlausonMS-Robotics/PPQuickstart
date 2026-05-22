package org.firstinspires.ftc.teamcode.simplifiedpathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.Action;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.SimpleActions.*;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimpleBezier;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

@Autonomous(name = "Simple Action Auto", group = "Simplified")
public class SimpleActionAuto extends LinearOpMode {
    SimplifiedRobot robot = new SimplifiedRobot();
    SimplifiedFollower follower;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new SimplifiedFollower(robot);

        // Define a curve
        SimpleBezier curve = new SimpleBezier(
            new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0),
            new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 90)
        );

        // Create a sequence of actions
        Action autonomous = new SequentialAction(
            new InstantAction(() -> robot.motors.intakeMotor.setPower(1.0)), // Start intake
            new WaitAction(1.0),                                            // Wait for 1s
            new PathAction(follower, robot, curve),                         // Drive the curve
            new InstantAction(() -> robot.motors.intakeMotor.setPower(0)),  // Stop intake
            new TimedAction(() -> robot.motors.transferMotor.setPower(1.0), 2.0), // Run transfer for 2s
            new InstantAction(() -> robot.motors.transferMotor.setPower(0)) // Stop transfer
        );

        waitForStart();

        // Run the actions in the main loop
        while (opModeIsActive() && !autonomous.run()) {
            telemetry.addData("Status", "Running Actions");
            telemetry.update();
        }

        follower.stop();
        telemetry.addData("Status", "Complete");
        telemetry.update();
    }
}
