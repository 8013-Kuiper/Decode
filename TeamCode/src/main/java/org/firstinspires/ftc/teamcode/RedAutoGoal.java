package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous (name = "Blue Side Goal")
public class RedAutoGoal extends LinearOpMode {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    private Follower follower;
    private Timer opmodeTimer;
    DcMotor turretRotatation;

    Servo gate;
    MotorEx launcher;
    Servo spindex;

    int targetRPM;

    private final Pose startPose = new Pose(117, 132, Math.toRadians(-144));
    private final Pose scorePose = new Pose(84,100, Math.toRadians(55));

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
        shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        Path alignPickup1 = new Path(new BezierLine(scorePose, new Pose(102, 84, Math.toRadians(0))));
        alignPickup1.setConstantHeadingInterpolation(Math.toRadians(55));

        initAprilTag();

        waitForStart();

        follower.followPath(shootPreload);

        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        double distance = -1;
        while (distance == -1) {
            distance = getDistance(true);
            if (distance < 50) {
                targetRPM = (int) (16 * distance + 3205);
            } else {
                targetRPM = (int) (16 * distance + 3155);
            }
            telemetry.addData("distance", distance);
            telemetry.addData("target RPM", targetRPM);
            telemetry.update();
        }

        launcher.setVelocity((((double) targetRPM /60*28)-750));
        sleep (1250);
        gate.setPosition(0);
        sleep (150);
        gate.setPosition(1);
        sleep(500);
        spindex.setPosition(1);
        sleep(900);
        sleep (500);
        gate.setPosition(0);
        sleep (150);
        gate.setPosition(1);
        sleep(500);

        follower.followPath(alignPickup1);
        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }
    }

    private void initAprilTag() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    private double getDistance(boolean isRed) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (isRed) {
            for (AprilTagDetection det : detections) {
                if (det.id == 24) {
                    return det.ftcPose.range;
                }
            }
        } else {
            for (AprilTagDetection det : detections) {
                if (det.id == 20) {
                    return det.ftcPose.range;
                }
            }
        }
        return -1.0;
    }
}
