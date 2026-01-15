package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@TeleOp
@Disabled
public class PowerFunctionTool extends OpMode {
    MotorEx launcher;
    Servo gate;

    int targetRPM = 3000;

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    List<Double> Distance = new ArrayList<Double>();
    List<Integer> RPM = new ArrayList<Integer>();
    String path = "FIRST/settings/PowerFunction.csv";

    boolean detectRed = false;

    @Override
    public void init() {
        initAprilTag();

        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        gate = hardwareMap.get(Servo.class, "gate");
    }
    public void start() {
        // set the run mode
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(20, 0, 0);
        launcher.setFeedforwardCoefficients(0.35, 0.35);
        gate.setPosition(1);
    }

    public void loop() {
        double distance = getDistance(detectRed);
        double currentRPM = launcher.getVelocity()/28*60;

        launcher.setVelocity(((double) targetRPM /60*28)+250);

        if (gamepad1.right_bumper) {
            gate.setPosition(0);
        } else {
            gate.setPosition(1);
        }

        if (gamepad1.a) {
            detectRed = true;
        } else if (gamepad1.b) {
            detectRed = false;
        }

        if (gamepad1.right_trigger > 0.1) {
            targetRPM += (int) (50*gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0.1) {
            targetRPM -= (int) (50*gamepad1.left_trigger);
        }

        if (gamepad2.y) {
            Distance.add(distance);
            RPM.add(targetRPM);
        }

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Distance from tag", distance);
        telemetry.addData("Number of data points", Distance.size());
        telemetry.addData("Detect red?", detectRed);
        telemetry.update();
    }

    public void stop() {
        try (FileWriter writer = new FileWriter(path, false)) {  // false = overwrite
            int max = Math.max(Distance.size(), RPM.size());
            for (int i = 0; i < max; i++) {
                String dist = (i < Distance.size()) ? String.valueOf(Distance.get(i)) : "";
                String rpmVal = (i < RPM.size()) ? String.valueOf(RPM.get(i)) : "";
                writer.append(dist).append(",").append(rpmVal).append("\n");
            }
        } catch (IOException e) {
            telemetry.addData("Error", e.toString());
            telemetry.update();
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
                    // compute Euclidean distance from tag pose translation (meters)
                    return det.ftcPose.range;
                }
            }
        } else {
            for (AprilTagDetection det : detections) {
                if (det.id == 20) {
                    // compute Euclidean distance from tag pose translation (meters)
                    return det.ftcPose.range;
                }
            }
        }
        return -1.0;
    }
}
