package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
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
    int count = 0;


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

        angleCorrected = (180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading())) + count*360;

        Telemetry.addData("angle corercted", angleCorrected);

        if (angleCorrected > 350) {
            count --;
        } else if (angleCorrected < -225) {
            count ++;
        }

        Telemetry.addData("count", count);

        target = (int) (angleCorrected*TicksPerDeg);

        turretRotation.setTargetPosition(target);

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
            Pose3D botpose_mt2 = result.getBotpose();
            if (botpose_mt2 != null) {
                double x = 72 - (botpose_mt2.getPosition().y * 39.37) + 3;
                double y = 72 + (-botpose_mt2.getPosition().x * 39.37) - 3;
                Telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
//                Telemetry.addData("Corrected MT2 Location:", "(" + (x + 72) + ", " + (y+72) + ")");
                currentPose = new Pose(x, y, Math.toRadians(robotYaw));
                follower.setPose(currentPose);
            } else {
                currentPose = follower.getPose();
            }
        } else {
            currentPose = follower.getPose();
        }

        Telemetry.addData("target angle", angleCorrected);
        Telemetry.addData("detect blue?", detectBlue);
        Telemetry.addData("target Pos", target);
        Telemetry.addData("currentPos", turretRotation.getCurrentPosition());
        Telemetry.addData("PosX", currentPose.getX());
        Telemetry.addData("PosY", currentPose.getY());
        Telemetry.addData("distance", follower.getPose().distanceFrom(blueGoal));
        Telemetry.update(telemetry);
    }
}
