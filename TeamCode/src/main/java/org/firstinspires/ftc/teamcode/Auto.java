package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous
public class Auto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    Servo elevator;
    DcMotorEx launcher;

    private final Pose startPose = new Pose(60, 50, Math.toRadians(0));
    private final Pose scorePose = new Pose(60,80, Math.toRadians(0));

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
    }
}
