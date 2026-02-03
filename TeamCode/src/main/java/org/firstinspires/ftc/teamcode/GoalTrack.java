package org.firstinspires.ftc.teamcode;


import com.bylazar.panels.Panels;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;

@TeleOp
public class GoalTrack extends OpMode {
    final double TicksPerDeg = 4.4546;

    Pose currentPose;
    Follower follower;
    boolean detectBlue=true;

    Pose blueGoal = new Pose(6,133);
    Pose redGoal = new Pose(144-6, 133 );

    DcMotor turretRotation;
    double angle;

    TelemetryManager Telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public void init () {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(currentPose);
        follower.update();

        turretRotation = hardwareMap.get(DcMotor.class, "turret");

        turretRotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        currentPose = new Pose(72,72, Math.toRadians(0));
    }

    @Override
    public void start() {
        if (detectBlue) {
            angle = Math.atan((currentPose.getX() - blueGoal.getX())/(currentPose.getY()) - blueGoal.getY());
        }

        follower.startTeleOpDrive();

        turretRotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretRotation.setTargetPosition((int) ((angle - follower.getPose().getHeading())/TicksPerDeg));
    }

    @Override
    public void loop() {
        if (detectBlue) {
            angle = Math.atan((currentPose.getX() - blueGoal.getX())/(currentPose.getY()) - blueGoal.getY());
        } else if (!detectBlue){
            angle = Math.atan((currentPose.getX() - blueGoal.getX())/(currentPose.getY()) - blueGoal.getY());
        }

        turretRotation.setTargetPosition((int) ((Math.toDegrees(angle - follower.getPose().getHeading()))/TicksPerDeg));

        follower.update();
        currentPose = follower.getPose();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        turretRotation.setPower(0.25);

        if (gamepad1.a){
            detectBlue = !detectBlue;
        }


        Telemetry.addData("angle", angle);
        Telemetry.addData("detect blue?", detectBlue);
        Telemetry.update(telemetry);
    }
}
