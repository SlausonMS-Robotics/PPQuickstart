package org.firstinspires.ftc.teamcode.simplifiedpathing.follower;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Use this OpMode to check your Pinpoint localization.
 * Push the robot around manually and verify the X and Y values 
 * match your measurements (use a ruler!).
 */
@TeleOp(name = "Simple Pinpoint Test", group = "Simplified")
public class SimpleLocalizationTest extends LinearOpMode {
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // TODO: Ensure these match your Autonomous configuration!
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); 
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        telemetry.addLine("Ready! Move robot and check X/Y/Heading.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();

            telemetry.addData("X (inches)", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y (inches)", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("\nPress A to reset position to (0,0,0)");
            
            if (gamepad1.a) {
                pinpoint.resetPosAndIMU();
            }
            
            telemetry.update();
        }
    }
}
