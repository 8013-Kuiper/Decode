package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.CoaxialPod;
import com.pedropathing.ftc.drivetrains.SwerveConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static final double mass = .5; //Kilograms
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(mass);

    public static SwerveConstants swerveConstants = new SwerveConstants()
            .maxPower(1); //determines the max power of the drivetrain
//      .zeroPowerBehavior(SwerveConstants.ZeroPowerBehavior.IGNORE_ANGLE_CHANGES)
    //the above disables x locking for swerve, which can be useful for tuning pod offsets

    private static CoaxialPod leftFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "leftFrontMotor", //the name of your motor in your config
                "leftFrontServo", //the name of your servo in your config
                "leftFrontEncoder", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0.01), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(353.1), //your pod's angle offset, in radians
                new Pose(0, 0), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0, //analog min voltage
                3.3, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightFront(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "leftFrontMotor", //the name of your motor in your config
                "leftFrontServo", //the name of your servo in your config
                "leftFrontEncoder", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0.01), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(353.1), //your pod's angle offset, in radians
                new Pose(0, 0), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0, //analog min voltage
                3.3, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod leftBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "leftFrontMotor", //the name of your motor in your config
                "leftFrontServo", //the name of your servo in your config
                "leftFrontEncoder", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0.01), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(353.1), //your pod's angle offset, in radians
                new Pose(0, 0), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0, //analog min voltage
                3.3, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }

    private static CoaxialPod rightBack(HardwareMap hardwareMap) {
        CoaxialPod pod = new CoaxialPod(
                hardwareMap,
                "leftFrontMotor", //the name of your motor in your config
                "leftFrontServo", //the name of your servo in your config
                "leftFrontEncoder", // the name of your analog encoder in your config
                new PIDFCoefficients(0.3, 0, 0.005, 0.01), //pod PIDF coefficients
                DcMotorSimple.Direction.FORWARD, //the direction of your motor
                DcMotorSimple.Direction.FORWARD, //the direction of your servo
                Math.toRadians(353.1), //your pod's angle offset, in radians
                new Pose(0, 0), //your pods x and y offsets,
                // in pedro coordinates (like with deadwheels)
                0, //analog min voltage
                3.3, //analog max voltage
                false); //encoder inverted
//  uncomment the below lines to change caching thresholds (by default 0.01)
//  pod.setMotorCachingThreshold(0.05);
//  pod.setServoCachingThreshold(0.05);
        return pod;
    }

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .swerveDrivetrain(swerveConstants,
                        leftFront(hardwareMap),
                        rightFront(hardwareMap),
                        leftBack(hardwareMap),
                        rightBack(hardwareMap))
                .build();
    }
}
