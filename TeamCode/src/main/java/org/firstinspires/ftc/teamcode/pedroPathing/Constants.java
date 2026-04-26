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
            .mass(13.5)
            .forwardZeroPowerAcceleration(-34.31)
            .lateralZeroPowerAcceleration(-67.6)

// TRANSLATIONAL
//            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.094, 0, 0.032, 0.008))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.012,0))

// HEADING
            .headingPIDFCoefficients(new PIDFCoefficients(2.0, 0, 0.15, 0))
//            .useSecondaryHeadingPIDF(true)
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.17,0,0.006,0))


//DRIVE
            //TODO - P USED TO BE 0.87
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.78,0.0,0.06,0.6,0.0))
//            .useSecondaryDrivePIDF(true)
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0,0.01,0.6,0.01))
            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 50, 1.5, 1.5);
//    public static PathConstraints pathConstraints = new PathConstraints(0.995, 40, 1.55, 1.6);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(80) //tuned was 76
            .yVelocity(62) //tuned was 58
            .useVoltageCompensation(true)
            .nominalVoltage(12.9);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5)
            .strafePodX(-1.34)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


}
