package org.firstinspires.ftc.teamcode.simplifiedpathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.simplifiedpathing.paths.SimplePathLoader;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.Action;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.RobotActionLibrary;
import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.SimpleActions.*;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimpleStates.State;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

import java.util.Map;

@Autonomous(name = "Library Action Auto", group = "Simplified")
public class SimplePPAuto extends LinearOpMode {
    SimplifiedRobot robot = new SimplifiedRobot();
    SimplifiedFollower follower;
    RobotActionLibrary actions;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new SimplifiedFollower(robot);
        actions = new RobotActionLibrary(robot);

        // 1. Load the individual paths from the .pp file
        Map<String, Action> paths = SimplePathLoader.loadPaths("paths1.pp", follower, robot);

        // 2. Build your manual sequence using the library
        Action autoSequence = new SequentialAction(
            actions.state(State.INTAKE_ON),        // Simple reusable action
            paths.get("Path 1"),
            actions.smartIntake(3.0),              // Complex "Macro" action
            paths.get("Path 2"),
            actions.spinAndShoot(3000, 2.0)       // Multi-step routine
        );

        waitForStart();

        while (opModeIsActive() && !autoSequence.run()) {
            telemetry.addData("Status", "Running Library Sequence");
            telemetry.update();
        }

        follower.stop();
    }
}
