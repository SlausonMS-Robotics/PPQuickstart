package org.firstinspires.ftc.teamcode.simplifiedpathing.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimpleBezier;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SimpleActions {

    /**
     * Follows a Bezier curve using the SimplifiedFollower.
     */
    public static class PathAction implements Action {
        private final SimplifiedFollower follower;
        private final SimplifiedRobot robot;
        private final SimpleBezier curve;
        private boolean initialized = false;

        public PathAction(SimplifiedFollower follower, SimplifiedRobot robot, SimpleBezier curve) {
            this.follower = follower;
            this.robot = robot;
            this.curve = curve;
        }

        @Override
        public boolean run() {
            if (!initialized) {
                follower.resetCurve();
                initialized = true;
            }
            robot.sensors.pinpoint.update();
            Pose2D currentPose = robot.sensors.pinpoint.getPosition();
            return follower.followCurve(currentPose, curve);
        }
    }

    /**
     * Runs a lambda once and finishes immediately.
     */
    public static class InstantAction implements Action {
        private final Runnable command;

        public InstantAction(Runnable command) {
            this.command = command;
        }

        @Override
        public boolean run() {
            command.run();
            return true;
        }
    }

    /**
     * Runs a lambda every loop for a specified duration.
     */
    public static class TimedAction implements Action {
        private final Runnable command;
        private final double durationSeconds;
        private ElapsedTime timer;

        public TimedAction(Runnable command, double durationSeconds) {
            this.command = command;
            this.durationSeconds = durationSeconds;
        }

        @Override
        public boolean run() {
            if (timer == null) timer = new ElapsedTime();
            command.run();
            return timer.seconds() >= durationSeconds;
        }
    }

    /**
     * Pauses for a specified duration.
     */
    public static class WaitAction implements Action {
        private final double durationSeconds;
        private ElapsedTime timer;

        public WaitAction(double durationSeconds) {
            this.durationSeconds = durationSeconds;
        }

        @Override
        public boolean run() {
            if (timer == null) timer = new ElapsedTime();
            return timer.seconds() >= durationSeconds;
        }
    }

    /**
     * Runs multiple actions in sequence.
     */
    public static class SequentialAction implements Action {
        private final List<Action> actions;
        private int currentIndex = 0;

        public SequentialAction(Action... actions) {
            this.actions = new ArrayList<>(Arrays.asList(actions));
        }

        @Override
        public boolean run() {
            if (currentIndex >= actions.size()) return true;
            
            boolean finished = actions.get(currentIndex).run();
            if (finished) {
                currentIndex++;
            }
            return currentIndex >= actions.size();
        }
    }

    /**
     * Runs multiple actions at the same time.
     * Finishes when ALL actions are done.
     */
    public static class ParallelAction implements Action {
        private final List<Action> actions;

        public ParallelAction(Action... actions) {
            this.actions = new ArrayList<>(Arrays.asList(actions));
        }

        @Override
        public boolean run() {
            boolean allFinished = true;
            for (Action action : actions) {
                if (!action.run()) {
                    allFinished = false;
                }
            }
            return allFinished;
        }
    }

    /**
     * Runs multiple actions at the same time.
     * Finishes as soon as ANY ONE action is done.
     */
    public static class RaceAction implements Action {
        private final List<Action> actions;

        public RaceAction(Action... actions) {
            this.actions = new ArrayList<>(Arrays.asList(actions));
        }

        @Override
        public boolean run() {
            for (Action action : actions) {
                if (action.run()) return true;
            }
            return false;
        }
    }
}
