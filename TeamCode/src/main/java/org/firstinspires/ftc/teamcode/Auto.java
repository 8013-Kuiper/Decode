package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous
public class Auto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    Servo elevator;
    DcMotorEx launcher;

    private final Pose startPose = new Pose(57, 135, Math.toRadians(270));
    private final Pose scorePose = new Pose(60,84, Math.toRadians(135));

    public void runOpMode() {
        elevator = hardwareMap.get(Servo.class, "gate");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        elevator.setPosition(0.5);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setPosition(0.5);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Path shootPreload = new Path(new BezierLine(startPose, scorePose));
        shootPreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        Path henryIsALoser = new Path(new BezierLine(scorePose, new Pose(50,50,Math.toRadians(135))));
        henryIsALoser.setConstantHeadingInterpolation(Math.toRadians(135));

        waitForStart();

        follower.followPath(shootPreload);

        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        launcher.setPower(1);
        elevator.setPosition(0);
        sleep(2500);
        elevator.setPosition(1);
        sleep(500);
        elevator.setPosition(0.5);

        follower.followPath(henryIsALoser);
        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }
    }
}
