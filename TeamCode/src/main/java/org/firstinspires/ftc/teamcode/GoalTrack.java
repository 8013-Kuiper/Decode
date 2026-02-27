package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class GoalTrack extends OpMode {
    final double driveTeeth = 59;
    final double drivenTeeth = 170;
    final double TicksPerDeg = 537.7*(drivenTeeth/driveTeeth)/360;

    Pose currentPose = new Pose(72, 72, 0);
    Follower follower;
    boolean detectBlue=true;

    Pose blueGoal = new Pose(6,135);
    Pose redGoal = new Pose(144-6, 135);

    Motor turretRotation;
    int target;
    double angle;
    double angleCorrected;
    double offset = 0;

    TelemetryManager Telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    Limelight3A limelight;

    public void init () {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(currentPose);
        follower.update();

        turretRotation = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);
        turretRotation.setRunMode(Motor.RunMode.PositionControl);
        turretRotation.setInverted(true);
        turretRotation.setPositionCoefficient(-0.05);
        turretRotation.setFeedforwardCoefficients(0.15,0.15);
        turretRotation.resetEncoder();
        turretRotation.setPositionTolerance(5);

        limelight = hardwareMap.get(Limelight3A.class, "camera");
        limelight.pipelineSwitch(0);
    }
    @Override
    public void start() {
        if (detectBlue) {
            angle = Math.atan2((currentPose.getX() - blueGoal.getX()), (currentPose.getY()) - blueGoal.getY());
        }

        follower.startTeleOpDrive();
        limelight.start();
    }

    @Override
    public void loop() {
        if (detectBlue) {
            angle = Math.atan2(blueGoal.getY() - currentPose.getY(), currentPose.getX() - blueGoal.getX());
        } else {
            angle = Math.atan2(redGoal.getY() - currentPose.getY(), currentPose.getX() - redGoal.getX());
        }

        target = (int) ((180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading()))*TicksPerDeg);

        angleCorrected = (180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading()));
        turretRotation.setTargetPosition((int) (target + offset));

        if (angleCorrected > 360) {
            offset -= 360*TicksPerDeg;
            angleCorrected -= 360;
        } else if (angleCorrected < -180) {
            offset += 360*TicksPerDeg;
            angleCorrected += 360;
        } else {
            turretRotation.setTargetPosition(target);
        }

        turretRotation.set(0.05);

        follower.update();
        currentPose = follower.getPose();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true);

        if (gamepad1.a) {
            detectBlue = !detectBlue;
        }

        Constants.currentPose = currentPose;

        LLResult result = limelight.getLatestResult();
        double robotYaw = Math.toDegrees(currentPose.getHeading());
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                Telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                Telemetry.addData("Corrected MT2 Location:", "(" + (x + 72) + ", " + (y+72) + ")");
                currentPose = new Pose(x+72, y+72, Math.toRadians(robotYaw));
            } else {
                currentPose = follower.getPose();
            }
        } else {
            currentPose = follower.getPose();
        }

        Telemetry.addData("angle", Math.toDegrees(angle));
        Telemetry.addData("target angle", (180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading())));
        Telemetry.addData("detect blue?", detectBlue);
        Telemetry.addData("target Pos", target);
        Telemetry.addData("currentPos", turretRotation.getCurrentPosition());
        Telemetry.addData("PosX", currentPose.getX());
        Telemetry.addData("PosY", currentPose.getY());
        Telemetry.addData("distance", follower.getPose().distanceFrom(blueGoal));
        Telemetry.update(telemetry);
    }
}
