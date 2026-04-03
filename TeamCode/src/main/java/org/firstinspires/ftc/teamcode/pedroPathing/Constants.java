package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Pose currentPose;
    public static boolean detectBlue;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.07)
            .forwardZeroPowerAcceleration(-45.56022)
            .lateralZeroPowerAcceleration(-86.73445)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.04, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5,0,0.1,0.01));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.75)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(47.2589)
            .yVelocity(36.7730);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("frontRight")
            .strafeEncoder_HardwareMapName("backRight")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            )
            .forwardPodY(-7.25)
            .strafePodX(-3)
            .forwardTicksToInches(0.00197760)
            .strafeTicksToInches(0.00197760);

//    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
//            .forwardTicksToInches(0.00197760)
//            .strafeTicksToInches(0.00197760)
//            .turnTicksToInches(0.00197760)
//            .leftPodY(7.5)
//            .rightPodY(-7.25)
//            .strafePodX(-3)
//            .leftEncoder_HardwareMapName("intake")
//            .rightEncoder_HardwareMapName("frontRight")
//            .strafeEncoder_HardwareMapName("backRight")
//            .leftEncoderDirection(Encoder.FORWARD)
//            .rightEncoderDirection(Encoder.REVERSE)
//            .strafeEncoderDirection(Encoder.REVERSE);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.8, 0.75);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
