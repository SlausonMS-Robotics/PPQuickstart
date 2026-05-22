package org.firstinspires.ftc.teamcode.simplifiedpathing.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimpleBezier;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimpleTurretAimer;

@Autonomous(name = "Simple Blue Solo Auto", group = "Simplified")
public class SimpleBlueSoloAuto extends LinearOpMode {
    SimplifiedRobot robot = new SimplifiedRobot();
    SimplifiedFollower follower;
    SimpleTurretAimer turretAimer;

    private int pathState = 0;
    private int robotState = 1;
    private ElapsedTime stateTimer = new ElapsedTime();
    private static final double shootPause = 3.0;
    private static final double midDist = 1.35;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new SimplifiedFollower(robot);
        turretAimer = new SimpleTurretAimer(robot);

        // Set starting pose
        robot.sensors.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 48, 8, AngleUnit.DEGREES, 90));

        // Define Paths
        // Path1: (48, 8, 90) -> (60, 84, 135)
        Pose2D p1_end = new Pose2D(DistanceUnit.INCH, 60, 84, AngleUnit.DEGREES, 135);
        
        // Path2: (60, 84, 135) -> (50, 82, 180)
        Pose2D p2_end = new Pose2D(DistanceUnit.INCH, 50, 82, AngleUnit.DEGREES, 180);
        
        // Path3: (50, 82, 180) -> (21, 82, 180)
        Pose2D p3_end = new Pose2D(DistanceUnit.INCH, 21, 82, AngleUnit.DEGREES, 180);
        
        // Path4: (21, 82, 180) -> (59.8, 84, 135)
        Pose2D p4_end = new Pose2D(DistanceUnit.INCH, 59.8, 84, AngleUnit.DEGREES, 135);
        
        // Path5 (Curve): (59.8, 84, 135), (47, 60), (48, 58, 180)
        SimpleBezier path5 = new SimpleBezier(
            new Pose2D(DistanceUnit.INCH, 59.8, 84, AngleUnit.DEGREES, 135),
            new Pose2D(DistanceUnit.INCH, 47, 60, AngleUnit.DEGREES, 157.5),
            new Pose2D(DistanceUnit.INCH, 48, 58, AngleUnit.DEGREES, 180)
        );
        
        // Path6: (48, 58, 180) -> (12, 58, 180)
        Pose2D p6_end = new Pose2D(DistanceUnit.INCH, 12, 58, AngleUnit.DEGREES, 180);
        
        // Path7 (Curve): (12, 58, 180), (50, 60), (60, 83.9, 135)
        SimpleBezier path7 = new SimpleBezier(
            new Pose2D(DistanceUnit.INCH, 12, 58, AngleUnit.DEGREES, 180),
            new Pose2D(DistanceUnit.INCH, 50, 60, AngleUnit.DEGREES, 157.5),
            new Pose2D(DistanceUnit.INCH, 60, 83.9, AngleUnit.DEGREES, 135)
        );
        
        // Path8 (Curve): (60, 83.9, 135), (50, 64), (48, 36, 180)
        SimpleBezier path8 = new SimpleBezier(
            new Pose2D(DistanceUnit.INCH, 60, 83.9, AngleUnit.DEGREES, 135),
            new Pose2D(DistanceUnit.INCH, 50, 64, AngleUnit.DEGREES, 157.5),
            new Pose2D(DistanceUnit.INCH, 48, 36, AngleUnit.DEGREES, 180)
        );
        
        // Path9: (48, 36, 180) -> (12, 36, 180)
        Pose2D p9_end = new Pose2D(DistanceUnit.INCH, 12, 36, AngleUnit.DEGREES, 180);
        
        // Path10: (12, 36, 180) -> (60, 84, 135)
        Pose2D p10_end = new Pose2D(DistanceUnit.INCH, 60, 84, AngleUnit.DEGREES, 135);

        telemetry.addLine("Initialized Blue Solo Auto");
        telemetry.update();

        waitForStart();
        
        turretAimer.setShooterByDistance(midDist);
        robotState = 1; // Intake on

        while (opModeIsActive()) {
            robot.sensors.pinpoint.update();
            Pose2D currentPose = robot.sensors.pinpoint.getPosition();
            handleRobotState();

            switch (pathState) {
                case 0: // Drive to shooting position
                    if (follower.update(currentPose, p1_end)) {
                        setPathState(1);
                    }
                    break;
                case 1: // Shoot + wait
                    shootSequence(2);
                    break;
                case 2: // Drive to grab ball set 1 (Path 2)
                    if (follower.update(currentPose, p2_end)) {
                        setPathState(3);
                        robotState = 1;
                    }
                    break;
                case 3: // Grab set 1 (Path 3)
                    if (follower.update(currentPose, p3_end)) {
                        setPathState(4);
                    }
                    break;
                case 4: // Drive back to shoot (Path 4)
                    if (follower.update(currentPose, p4_end)) {
                        setPathState(5);
                    }
                    break;
                case 5: // Shoot + wait
                    shootSequence(6);
                    break;
                case 6: // Move to ball set 2 (Path 5 Curve)
                    if (follower.followCurve(currentPose, path5)) {
                        setPathState(7);
                        robotState = 1;
                    }
                    break;
                case 7: // Pick up set 2 (Path 6)
                    if (follower.update(currentPose, p6_end)) {
                        setPathState(8);
                    }
                    break;
                case 8: // Drive to shoot pos (Path 7 Curve)
                    if (follower.followCurve(currentPose, path7)) {
                        setPathState(9);
                    }
                    break;
                case 9: // Shoot + wait
                    shootSequence(10);
                    break;
                case 10: // Pickup set 3 (Path 8 Curve)
                    if (follower.followCurve(currentPose, path8)) {
                        setPathState(11);
                        robotState = 1;
                    }
                    break;
                case 11: // Grab set 3 (Path 9)
                    if (follower.update(currentPose, p9_end)) {
                        setPathState(12);
                    }
                    break;
                case 12: // Drive to shoot (Path 10)
                    if (follower.update(currentPose, p10_end)) {
                        setPathState(13);
                    }
                    break;
                case 13: // Final shoot
                    shootSequence(14);
                    break;
                case 14:
                    follower.stop();
                    robotState = 0;
                    break;
            }

            telemetry.addData("Path State", pathState);
            telemetry.addData("X", currentPose.getX(DistanceUnit.INCH));
            telemetry.addData("Y", currentPose.getY(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    private void setPathState(int state) {
        pathState = state;
        stateTimer.reset();
        follower.resetCurve();
    }

    private void shootSequence(int nextState) {
        if (stateTimer.seconds() < 0.5) {
            robotState = 1; // Prepare
        } else if (stateTimer.seconds() < shootPause) {
            robotState = 2; // Shoot
        } else {
            setPathState(nextState);
        }
    }

    private void handleRobotState() {
        switch (robotState) {
            case 0: // Off
                robot.motors.intakeMotor.setPower(0);
                robot.motors.transferMotor.setPower(0);
                robot.servos.setIntakeServos(false);
                break;
            case 1: // Intake On
                robot.motors.intakeMotor.setPower(1.0);
                robot.motors.transferMotor.setPower(0.25);
                robot.servos.setIntakeServos(true);
                break;
            case 2: // Shooting
                robot.motors.intakeMotor.setPower(1.0);
                robot.motors.transferMotor.setPower(1.0);
                robot.servos.setIntakeServos(true);
                break;
            case 3: // Hold
                robot.motors.intakeMotor.setPower(1.0);
                robot.motors.transferMotor.setPower(0);
                robot.servos.setIntakeServos(true);
                break;
        }
    }
}
