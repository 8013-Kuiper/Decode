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

@Autonomous (name = "")
public class FarAuto extends LinearOpMode {
    private Follower follower;
    private Timer opmodeTimer;

    private final Pose startPose = new Pose(81, 9, Math.toRadians(90));
    private final Pose finalPose = new Pose(100,15, Math.toRadians(90));

    public void runOpMode() {
        Constants.detectBlue = true;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Path shootPreload = new Path(new BezierLine(startPose, finalPose));
        shootPreload.setConstantHeadingInterpolation(Math.toRadians(90));

        waitForStart();

        follower.followPath(shootPreload);

        while (follower.isBusy()){
            sleep(50);
            follower.update();
        }

        Constants.currentPose = follower.getPose();
    }
}
