package org.firstinspires.ftc.teamcode.simplifiedpathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.simplifiedpathing.paths.SimplePathLoader;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.Action;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.SimpleActions.*;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

import java.util.Map;

@Autonomous(name = "Decoupled .PP Auto", group = "Simplified")
public class SimplePPAuto extends LinearOpMode {
    SimplifiedRobot robot = new SimplifiedRobot();
    SimplifiedFollower follower;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new SimplifiedFollower(robot);

        // 1. Load the individual paths from the .pp file into a map
        // The keys match the names you gave the paths in the .pp editor (e.g., "Path 1", "Path 2")
        Map<String, Action> paths = SimplePathLoader.loadPaths("paths1.pp", follower, robot);

        // 2. Build your manual sequence
        Action autoSequence = new SequentialAction(
            // Path 1 is BLOCKING (code waits here until Path 1 is done)
            paths.getOrDefault("Path 1", new InstantAction(() -> {})),
            
            // This section is NON-BLOCKING:
            // It runs "Path 2" AND the "Intake Sequence" at the same time.
            new ParallelAction(
                paths.getOrDefault("Path 2", new InstantAction(() -> {})),
                new SequentialAction(
                    new WaitAction(0.5), // Wait a half second into the drive
                    new InstantAction(() -> robot.motors.intakeMotor.setPower(1.0)),
                    new WaitAction(1.5),
                    new InstantAction(() -> robot.motors.intakeMotor.setPower(0))
                )
            ),
            
            new WaitAction(1.0),
            new InstantAction(() -> robot.motors.setShooterVelocity(0))
        );

        waitForStart();

        while (opModeIsActive() && !autoSequence.run()) {
            telemetry.addData("Status", "Running Decoupled Sequence");
            telemetry.update();
        }

        follower.stop();
    }
}
