package org.firstinspires.ftc.teamcode.simplifiedpathing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * An example Autonomous OpMode using the SimplifiedFollower and GoBilda Pinpoint.
 */
@Autonomous(name = "Simple Pinpoint Pathing", group = "Simplified")
public class SimplePathOpMode extends LinearOpMode {
    SimplifiedFollower follower;
    GoBildaPinpointDriver pinpoint;
    
    @Override
    public void runOpMode() {
        follower = new SimplifiedFollower(hardwareMap);
        
        // Initialize Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        
        // --- PINPOINT CONFIGURATION ---
        // TODO: Update these offsets for your specific robot!
        // X offset: sideways distance from tracking point to forward pod (Left = +, Right = -)
        // Y offset: forward distance from tracking point to strafe pod (Forward = +, Backward = -)
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); 
        
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);
        
        pinpoint.resetPosAndIMU();
        // ------------------------------

        telemetry.addLine("Ready! Pinpoint Initialized.");
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

    /**
     * Loops until the robot reaches the target pose using Pinpoint data.
     */
    private void runToPose(Pose2D target) {
        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();
            
            boolean reached = follower.update(currentPose, target);
            
            telemetry.addData("Target", target.getX(DistanceUnit.INCH) + ", " + target.getY(DistanceUnit.INCH));
            telemetry.addData("Current", currentPose.getX(DistanceUnit.INCH) + ", " + currentPose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", currentPose.getHeading(AngleUnit.DEGREES));
            telemetry.update();

            if (reached) break;
        }
    }
}
