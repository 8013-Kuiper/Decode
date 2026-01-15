package org.firstinspires.ftc.teamcode;

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

@Autonomous (name = "Blue Side Goal")
public class BlueAutoGoal extends LinearOpMode {
    private Follower follower;
    private Timer opmodeTimer;
    DcMotor turretRotatation;

    Servo gate;
    DcMotorEx launcher;

    private final Pose startPose = new Pose(27, 132, Math.toRadians(324));
    private final Pose scorePose = new Pose(60,85, Math.toRadians(135));

    public void runOpMode() {
        gate = hardwareMap.get(Servo.class, "gate");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        turretRotatation = hardwareMap.get(DcMotor.class, "turret");

        turretRotatation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        gate.setPosition(1);

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Path shootPreload = new Path(new BezierLine(startPose, scorePose));
        shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        Path alignPickup1 = new Path(new BezierLine(scorePose, new Pose(42, 84, Math.toRadians(180))));
        alignPickup1.setConstantHeadingInterpolation(Math.toRadians(135));

        waitForStart();

        follower.followPath(shootPreload);

        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

//        launcher.setPower(1);
//        gate.setPosition(0);
        sleep(2500);
//        gate.setPosition(1);
        sleep(500);
//        gate.setPosition(0.5);

        follower.followPath(alignPickup1);
        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }
    }
}
