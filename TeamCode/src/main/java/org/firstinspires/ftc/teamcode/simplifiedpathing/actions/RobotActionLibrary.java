package org.firstinspires.ftc.teamcode.simplifiedpathing.actions;

import org.firstinspires.ftc.teamcode.simplifiedpathing.actions.SimpleActions.*;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimpleStates.State;

/**
 * A central library for reusable robot actions and complex routines.
 */
public class RobotActionLibrary {
    private final SimplifiedRobot robot;

    public RobotActionLibrary(SimplifiedRobot robot) {
        this.robot = robot;
    }

    /**
     * An action that simply sets a robot state and finishes immediately.
     */
    public Action state(State state) {
        return new InstantAction(() -> robot.states.setState(state));
    }

    /**
     * A complex, multi-step routine that can be reused in any auto.
     * Example: Start shooter, wait for it to spin up, then shoot for 2 seconds.
     */
    public Action spinAndShoot(double rpm, double duration) {
        return new SequentialAction(
            new InstantAction(() -> robot.motors.setShooterVelocity(rpm * 28.0 / 60.0)),
            new WaitAction(1.0), // Spin up time
            state(State.SHOOTING),
            new WaitAction(duration),
            state(State.HOLD_BALL),
            new InstantAction(() -> robot.motors.setShooterVelocity(0))
        );
    }

    /**
     * Intake routine: sets state to intake, waits until a ball is detected or timeout.
     */
    public Action smartIntake(double timeout) {
        return new RaceAction(
            state(State.INTAKE_ON),
            new Action() {
                @Override
                public boolean run() {
                    return robot.sensors.getIntakeDistanceMM() < 50;
                }
            },
            new WaitAction(timeout)
        );
    }
}
