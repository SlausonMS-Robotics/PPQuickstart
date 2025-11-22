package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.PoseStorage;
import org.firstinspires.ftc.teamcode.robot.TurretAiming;
import org.firstinspires.ftc.teamcode.robot.limelight3A;
import org.firstinspires.ftc.teamcode.robot.motors;
import org.firstinspires.ftc.teamcode.robot.other_helpers;
import org.firstinspires.ftc.teamcode.robot.servos;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

@TeleOp(name = "Shooter Range Test", group = "23609")
public class RangeTest extends OpMode {

    // ---- Hardware ----
    private motors robotMotors = new motors();
    private servos Servos = new servos();
    private limelight3A limelight = new limelight3A();

    // ---- Helpers & Timers ----
    private final other_helpers distanceAverage = new other_helpers();
    private final ElapsedTime llTimer = new ElapsedTime();
    private final ElapsedTime debounceTimer = new ElapsedTime();

    // ---- State & Constants ----
    private double ll_goal_dist = 0;
    private int shooterSpeedAdjust = 0;
    private static final int MOVING_AVERAGE_SIZE = 1;
    private String myAllianceColor = "blue";
    private static final int ticks_per_rev = 28;
    private static final String SHOOTER_DATA_FILE = "/sdcard/FIRST/ShooterRangeData.csv";
    private TurretAiming turretAimer;
    Follower follower;

    @Override
    public void init() {
        // Initialize hardware
        Servos.init(hardwareMap);
        robotMotors.init(hardwareMap, Servos);
        limelight.init(hardwareMap, 0, telemetry);
        follower = Constants.createFollower(hardwareMap);
        Pose startPose = PoseStorage.currentPose;
        follower.setStartingPose(startPose);
// Instantiate our new reusable classes
        turretAimer = new TurretAiming(follower, limelight, Servos, robotMotors, telemetry);

        // Initialize helpers
        distanceAverage.initMovingAverage(MOVING_AVERAGE_SIZE);
        llTimer.reset();
        debounceTimer.reset();

        telemetry.addData("Status", "Initialization Complete");
        telemetry.addData(">>>", "Point at an AprilTag to begin range test.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Limelight Polling (20Hz) ---
        if (llTimer.milliseconds() > 50) {
            LLResult result = limelight.limelight.getLatestResult();
            if (result.isValid()) {
                ll_goal_dist = distanceAverage.updateAndGetAverage(result.getBotposeAvgDist());
                servos.setLedColor(servos.LedColor.GREEN);
            } else {
                // If no target, reset distance and clear the average
                ll_goal_dist = 0;
                distanceAverage.clearReadings();
                servos.setLedColor(servos.LedColor.RED);
            }
            llTimer.reset();
        }

        // --- Shooter RPM Calculation ---
        double targetRPM = 0;
        if (ll_goal_dist > 0.1) {
            targetRPM = other_helpers.FlywheelShooter.getRPMForDistance(ll_goal_dist);
        }

        // Convert target RPM to motor velocity (ticks/sec) and add manual adjustment
        double targetVelocity = (targetRPM + shooterSpeedAdjust) * ticks_per_rev / 60;
        robotMotors.setShooterVelocity(targetVelocity);

        turretAimer.update(myAllianceColor);

        // --- Gamepad Controls ---
        // Manual RPM Adjustment
        if (debounceTimer.milliseconds() > 350) {
            if (gamepad1.a || gamepad1.y){
                robotMotors.toggleIntake();
                debounceTimer.reset();
            }
            if (gamepad1.dpad_up) {
                shooterSpeedAdjust += 50;
                debounceTimer.reset();
            } else if (gamepad1.dpad_down) {
                shooterSpeedAdjust -= 50;
                debounceTimer.reset();
            } else if (gamepad1.x) {
                shooterSpeedAdjust = 0; // Reset adjustment
                debounceTimer.reset();
            }

            // Save Data Point

        }

        // Shooting
        if (gamepad1.right_trigger > 0.2) {
            robotMotors.shoot();
        }
        else {
            if(robotMotors.isIntakeOn()) {
                robotMotors.transferBackwards(); //turn transfer off
            }
            else robotMotors.setTransfer(false);
        }


        // --- Telemetry ---
        telemetry.addData("Limelight Distance (m)", ll_goal_dist);
        telemetry.addData("Target Base RPM", targetRPM);
        telemetry.addData("Manual RPM Adjustment", shooterSpeedAdjust);
        telemetry.addData("Final Target RPM", targetRPM + shooterSpeedAdjust);
        telemetry.addData("Final Target Velocity (ticks/s)", targetVelocity);
        telemetry.addLine("\n--- Controls ---");
        telemetry.addData("DPad Up/Down", "Adjust RPM by +/- 50");
        telemetry.addData("X Button", "Reset RPM Adjustment");
        telemetry.addData("Right Trigger", "Shoot");
        telemetry.addData("Start", "Save Data Point");
        telemetry.update();
    }
}