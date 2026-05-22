package org.firstinspires.ftc.teamcode.simplifiedpathing.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimpleTurretAimer;
import org.firstinspires.ftc.teamcode.simplifiedpathing.follower.SimplifiedFollower;
import org.firstinspires.ftc.teamcode.simplifiedpathing.robot.SimplifiedRobot;

@TeleOp(name = "Simple Auto Aim Teleop", group = "Simplified")
public class SimpleAutoAimTeleop extends LinearOpMode {
    SimplifiedRobot robot = new SimplifiedRobot();
    SimplifiedFollower follower;
    SimpleTurretAimer turretAimer;

    private int robotState = 1; // 0: Off, 1: Intake, 2: Shooting, 3: Hold, 4: Slow Intake
    private boolean isLocked = false;
    private ElapsedTime lockTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new SimplifiedFollower(robot);
        turretAimer = new SimpleTurretAimer(robot);

        waitForStart();

        while (opModeIsActive()) {
            // Update Localization
            robot.sensors.pinpoint.update();
            Pose2D pose = robot.sensors.pinpoint.getPosition();

            // Drive Control (Field Centric)
            double speedScalar = gamepad1.left_trigger > 0.2 ? 1.0 : 0.5;
            follower.driveFieldCentric(
                -gamepad1.left_stick_y * speedScalar,
                -gamepad1.left_stick_x * speedScalar,
                -gamepad1.right_stick_x * speedScalar,
                pose.getHeading(AngleUnit.RADIANS)
            );

            // Intake Logic
            if (gamepad1.y) {
                robot.motors.intakeMotor.setPower(-1.0);
            } else {
                handleRobotState();
            }

            // Aiming Logic
            isLocked = turretAimer.updateLimelightAiming();
            
            LLResult result = robot.limelight.getResult();
            if (result != null && result.isValid()) {
                //turretAimer.setShooterByDistance(result.getBotposeAvgDist());
            }

            // Shooting Logic
            if (gamepad1.right_trigger > 0.2) {
                if (isLocked || lockTimer.seconds() > 0.75) {
                    robotState = 2; // Shooting
                }
            } else {
                if (robot.sensors.getIntakeDistanceMM() < 50) {
                    robotState = 3; // Hold (ball detected)
                } else {
                    robotState = 1; // Standard Intake
                }
                lockTimer.reset();
            }

            // Telemetry
            telemetry.addData("State", robotState);
            telemetry.addData("Locked", isLocked);
            telemetry.addData("X", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pose.getY(DistanceUnit.INCH));
            telemetry.update();
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
