package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous (name = "Blue Side Goal")
public class BlueAutoGoal extends LinearOpMode {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    private Follower follower;
    private Timer opmodeTimer;
    DcMotor turretRotatation;

    Servo gate;
    MotorEx launcher;
    Servo spindex;

    int targetRPM;

    private final Pose startPose = new Pose(27, 132, Math.toRadians(-36));
    private final Pose scorePose = new Pose(58,90, Math.toRadians(135));

    public void runOpMode() {
        gate = hardwareMap.get(Servo.class, "gate");
        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        turretRotatation = hardwareMap.get(DcMotor.class, "turret");
        spindex = hardwareMap.get(Servo.class, "rotate");

        turretRotatation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindex.setPosition(0.5);
        gate.setPosition(1);

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(20, 0, 0);
        launcher.setFeedforwardCoefficients(0.35, 0.35);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Path shootPreload = new Path(new BezierLine(startPose, scorePose));
        shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180));

        Path alignPickup1 = new Path(new BezierLine(scorePose, new Pose(50, 85, Math.toRadians(180))));
        alignPickup1.setConstantHeadingInterpolation(Math.toRadians(180));

        waitForStart();

        follower.followPath(shootPreload);

        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        follower.followPath(alignPickup1);
        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        follower.followPath(new PathChain(
                new Path(new BezierLine(
                        new Pose(50, 85, Math.toRadians(180)),
                        new Pose(25, 85, Math.toRadians(180))))),
                0.2,
                true);
        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        follower.followPath(new PathChain(
                new Path(new BezierLine(
                        new Pose(25, 85, Math.toRadians(180)),
                        scorePose))),
        0.2, true);
    }
}
