package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8)
            .forwardZeroPowerAcceleration(-49)
            .lateralZeroPowerAcceleration(-73)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(true)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.012, 0, 0.0005, 0.1))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.15, 0.1))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.01, 0, 0.001, 0.47, 0.1)
            )
            .secondaryTranslationalPIDFCoefficients(
                    new PIDFCoefficients(0.01, 0, 0.0005, 0.1)
            )
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.15, 0))
            .secondaryDrivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.008, 0, 0.001, 0.45, 0.1)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("ehmotor1")
            .leftRearMotorName("ehmotor0")
            .rightFrontMotorName("ehmotor2")
            .rightRearMotorName("ehmotor3")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(61.07)
            .yVelocity(55.1);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7.75)
            .strafePodX(2)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
            )
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            50,
            1.25,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}



