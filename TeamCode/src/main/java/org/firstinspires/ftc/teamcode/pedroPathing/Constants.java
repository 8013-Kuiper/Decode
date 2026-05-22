package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static final double mass = 6.80389; //Kilograms

    public static final double dtLength = 12.780;
    public static final double dtWidth = 17.450000;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(mass);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .maxPower(1) //determines the max power of the drivetrain
      .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES);
    //the above disables x locking for swerve, which can be useful for tuning pod offsets

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "frontLeft", //the name of your motor in your config
                "frontLeftS", //the name of your servo in your config
                "frontLeftA", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(136.45070), //your pod's angle offset, in radians
                new Pose(97.281, 156.615), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0.017, //analog min voltage
                3.212, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "frontRight", //the name of your motor in your config
                "frontRightS", //the name of your servo in your config
                "frontRightA", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(327.344), //your pod's angle offset, in radians
                new Pose(97.281000, -156.615000), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0.019, //analog min voltage
                3.216, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "backLeft", //the name of your motor in your config
                "backLeftS", //the name of your servo in your config
                "backLeftA", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(279.089), //your pod's angle offset, in radians
                new Pose(-97.281000, 156.615000), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0.02, //analog min voltage
                3.228, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "backRight", //the name of your motor in your config
                "backRightS", //the name of your servo in your config
                "backRightA", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(98.743455), //your pod's angle offset, in radians
                new Pose(-97.281000, -156.615000), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0.008, //analog min voltage
                3.446, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(1)
            .rightPodY(-1)
            .strafePodX(-2.5)
            .leftEncoder_HardwareMapName("frontLeft")
            .rightEncoder_HardwareMapName("backRight")
            .strafeEncoder_HardwareMapName("backLeft")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants,
                        leftFront(hardwareMap),
                        rightFront(hardwareMap),
                        leftBack(hardwareMap),
                        rightBack(hardwareMap))
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}
