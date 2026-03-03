/*
package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
*/ //Old Libraries
/*CHANGES:
Lines 91, 92, 210, 223, 339, 340
*/

package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Red Side Goal")
public class RedAutoGoal extends LinearOpMode {
    /*
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
    private final Pose scorePose = new Pose(86,84, Math.toRadians(55));
    */ //Original Code

    final double driveTeeth = 59;
    final double drivenTeeth = 170;
    final double TicksPerDeg = 537.7*(drivenTeeth/driveTeeth)/360;

    private Follower follower;
    private Timer opmodeTimer;

    Motor turretRotation;;
    Servo gate;
    MotorEx launcher;
    CRServo spin;
    DcMotor intake;

    int target;

    double distance = 50;

    int targetRPM;

    private final Pose startPose = new Pose((144-27), 132, Math.toRadians(180+36));
    private final Pose scorePose = new Pose((144-58),90, Math.toRadians(45));

    TelemetryManager Telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public void runOpMode() {
        /*
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

        launcher.setVelocity((2600/60*28)-1000);
        sleep (1250);
        gate.setPosition(0);
        sleep (150);
        gate.setPosition(1);
        sleep(500);
        spindex.setPosition(1);
        sleep(850);
        spindex.setPosition(0.5);
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
    */ //Original Code
        Constants.detectBlue = false;

        gate = hardwareMap.get(Servo.class, "gate");
        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        spin = hardwareMap.get(CRServo.class, "rotate");
        intake = hardwareMap.get(DcMotor.class, "intake");

        turretRotation = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);
        turretRotation.setRunMode(Motor.RunMode.PositionControl);
        turretRotation.setInverted(true);
        turretRotation.setPositionCoefficient(-0.05);
        turretRotation.setFeedforwardCoefficients(0.15,0.15);
        turretRotation.resetEncoder();
        turretRotation.setPositionTolerance(5);

        //spin.setPosition(0.5);
        gate.setPosition(1);

        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(20, 0, 0);
        launcher.setFeedforwardCoefficients(0.35, 0.5);
        launcher.setInverted(true);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Path shootPreload = new Path(new BezierLine(startPose, scorePose));
        shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), Math.toRadians(180));

        Path alignPickup1 = new Path(new BezierLine(scorePose, new Pose((144-50), 85, Math.toRadians(0))));
        alignPickup1.setConstantHeadingInterpolation(Math.toRadians(0));

        waitForStart();

        follower.followPath(shootPreload);

        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }


        target = (int) (-26.5*TicksPerDeg);

        turretRotation.setTargetPosition(target);

        while (!turretRotation.atTargetPosition()){
            turretRotation.set(0.1);
        }
        turretRotation.set(0);

        targetRPM = 2000;
        //targetRPM = (int) (10.6 * distance);

        //launcher.setVelocity((((double) targetRPM /60*28)));

        launcher.setVelocity(((130)));
//       while ((launcher.getVelocity()/28*60) < targetRPM-75 || (launcher.getVelocity()/28*60) > targetRPM+75){
//            launcher.setVelocity((((double) targetRPM /60*28)));
//            Telemetry.addData("current RPM", launcher.getVelocity()/28*60);
//            Telemetry.addData("target", targetRPM);
//            Telemetry.update(telemetry);
//        }
        sleep(1500);
        Telemetry.addData("current RPM", launcher.getVelocity()/28*60);
        Telemetry.addData("target", targetRPM);
        Telemetry.update(telemetry);
        //shoot first

        gate.setPosition(0);
        sleep(650);
        gate.setPosition(1);

        sleep(500);

        //load second

        //spin.setPosition(1);
        spin.setPower(1.0);
        intake.setPower(-1.0);
        sleep(1100);
        intake.setPower(0.0);
        //spin.setPosition(0.5);
        //spin.setPower(-.3);
//        sleep(50);
//        spin.setPosition(-0.5);
//        sleep(50);
//        spin.setPosition(0.5);
        sleep(50);
        spin.setPower(0);

        //targetRPM = (int) (10.6 * distance + 3100);
        /*
        while ((launcher.getVelocity()/28*60) < targetRPM-75 || (launcher.getVelocity()/28*60) > targetRPM+75){
            launcher.setVelocity((((double) targetRPM /60*28)+300));
            Telemetry.addData("current RPM", launcher.getVelocity()/28*60);
            Telemetry.addData("target", targetRPM);
            Telemetry.update(telemetry);
        }
         */
        sleep(250);

        //shoot second
        gate.setPosition(0);
        sleep(500);
        gate.setPosition(1);

        sleep(500);
//
        //load third
        //spin.setPosition(1);
        spin.setPower(1.0);
        intake.setPower(-1.0);
        sleep(1100);
        intake.setPower(0.0);
        //spin.setPosition(0.5);
//        sleep(50);
//        spin.setPosition(-0.5);
//        sleep(50);
//        spin.setPosition(0.5);
        sleep(50);
        spin.setPower(0.0);

        //targetRPM = (int) (10.6 * distance + 3100);

        /*
        while ((launcher.getVelocity()/28*60) < targetRPM-75 || (launcher.getVelocity()/28*60) > targetRPM+75){
            launcher.setVelocity((((double) targetRPM /60*28)+300));
            Telemetry.addData("current RPM", launcher.getVelocity()/28*60);
            Telemetry.addData("target", targetRPM);
            Telemetry.update(telemetry);
        }
        */
        sleep(250);

        //shoot second

        gate.setPosition(0);
        sleep(500);
        gate.setPosition(1);

        sleep(500);

//        launcher.setVelocity(0);
//        sleep(150);


        follower.followPath(alignPickup1);
        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        intake.setPower(-1);
        spin.setPower(1);

        follower.followPath(new PathChain(
                        new Path(new BezierLine(
                                new Pose((144-50), 85, Math.toRadians(0)),
                                new Pose((144-25), 85, Math.toRadians(0))))),
                0.2,
                true);
        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        intake.setPower(0);
        //spin.setPosition(0.5);
//
//        follower.followPath(new PathChain(
//                new Path(new BezierLine(
//                        new Pose(25, 85, Math.toRadians(180)),
//                        scorePose))),
//        0.8, true);
//
//        while (follower.isBusy()){
//            sleep(50);
//            follower.update();
//        }
//
//        target = (int) (110*TicksPerDeg);
//
//        turretRotation.setTargetPosition(target);
//
//        while (!turretRotation.atTargetPosition()){
//            turretRotation.set(0.1);
//        }
//        turretRotation.set(0);
//
//        spin.setPosition(-1);
//        intake.setPower(1);
//        sleep(150);
//        intake.setPower(0);
//        sleep(250);
//        spin.setPosition(0.5);
////
////        follower.followPath(new Path(
////                new BezierLine(
////                        scorePose,
////                        new Pose(48,72)
////                )
////        ));
////
////        while (follower.isBusy()){
////            sleep(50);
////            follower.update();
////        }
////
        Constants.currentPose = follower.getPose();
    }
/*
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
    */ //getDistance Method
}
