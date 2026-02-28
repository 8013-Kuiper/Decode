package org.firstinspires.ftc.teamcode.Unused;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp (group = "Test")
//@Disabled
public class PowerFunctionTool extends OpMode {
    MotorEx launcher;
    Servo gate;

    DcMotor intake;
    Servo spin;

    int targetRPM = 0;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    List<Double> Distance = new ArrayList<>();
    List<Double> RPM = new ArrayList<>();

    boolean detectRed = false;
    Follower follower;
    Pose currentPose = new Pose(72, 72, 0);

    Pose blueGoal = new Pose(6,135);
    Pose redGoal = new Pose(144-6, 135);

    double distance = 0;

    @Override
    public void init() {
//        initAprilTag();

        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        launcher.setInverted(true);
        gate = hardwareMap.get(Servo.class, "gate");

        spin = hardwareMap.get(Servo.class, "rotate");
        intake = hardwareMap.get(DcMotor.class, "intake");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(currentPose);
        follower.update();
    }
    public void start() {
        // set the run mode
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(20, 0, 0);
        launcher.setFeedforwardCoefficients(0.35, 0.35);
        gate.setPosition(1);
    }

    public void loop() {
        double currentRPM = launcher.getVelocity()/28*60;

        launcher.setVelocity(((double) targetRPM /60*28)+250);

        if (gamepad1.right_bumper) {
            gate.setPosition(0);
        } else {
            gate.setPosition(1);
        }

        //Transfer Controls
        if (gamepad1.a) {
            spin.setPosition(1.0);
        } else if (gamepad1.b) {
            spin.setPosition(0);
        } else {
            spin.setPosition(0.5);
        }

        //Intake Controls
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(gamepad1.left_trigger);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad1.a) {
            detectRed = true;
        } else if (gamepad1.b) {
            detectRed = false;
        }

        if (detectRed) {
            distance = follower.getPose().distanceFrom(redGoal);
        } else {
            distance = follower.getPose().distanceFrom(blueGoal);
        }

        follower.update();

//        if (distance < 50) {
//            targetRPM = (int) (16*distance + 3205);
//        } else {
//            targetRPM = (int) (16 * distance + 3155);
//        }
        if (gamepad1.right_trigger > 0.1) {
            targetRPM += (int) (50*gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            targetRPM -= (int) (50*gamepad1.left_trigger);
        }

        if (gamepad1.b) {
            Distance.add(distance);
            RPM.add((double) targetRPM);
        }


        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Distance from tag", distance);
        telemetry.addData("Detect red?", detectRed);
        telemetry.addData("Equation", "y = " + calculateB(Distance, RPM) + "x + " + calculateA(Distance, RPM));
        telemetry.update();
    }

    private double calculateA (List<Double> distance, List<Double> rpm) {
        double sumx = 0;
        double sumy = 0;
        double sumxy = 0;
        double sumx2 = 0;
        double sumy2 = 0;
        double n = distance.size();

        double a = 0;

        for (int i=0; i<distance.size(); i++) {
            sumx += distance.get(i);
        }
        for (int i=0; i<rpm.size(); i++) {
            sumy += rpm.get(i);
        }

        for (int i=0; i<distance.size();i++) {
            sumxy += distance.get(i) * rpm.get(i);
        }
        for (int i=0; i<distance.size(); i++) {
            sumx2 += distance.get(i) * distance.get(0);
        }
        for (int i=0; i<rpm.size(); i++) {
            sumy2 += rpm.get(i) * rpm.get(i);
        }

        return ((sumy*sumx2) - (sumx*sumxy))/((n*sumx2) - (sumx*sumx));
    }

    private double calculateB (List<Double> distance, List<Double> rpm) {
        double sumx = 0;
        double sumy = 0;
        double sumxy = 0;
        double sumx2 = 0;
        double sumy2 = 0;
        double n = distance.size();

        double a = 0;

        for (int i=0; i<distance.size(); i++) {
            sumx += distance.get(i);
        }
        for (int i=0; i<rpm.size(); i++) {
            sumy += rpm.get(i);
        }

        for (int i=0; i<distance.size();i++) {
            sumxy += distance.get(i) * rpm.get(i);
        }
        for (int i=0; i<distance.size(); i++) {
            sumx2 += distance.get(i) * distance.get(0);
        }
        for (int i=0; i<rpm.size(); i++) {
            sumy2 += rpm.get(i) * rpm.get(i);
        }

        return ((n*sumxy) - (sumx*sumy))/((n*sumx2) - (sumx*sumx));
    }
}
