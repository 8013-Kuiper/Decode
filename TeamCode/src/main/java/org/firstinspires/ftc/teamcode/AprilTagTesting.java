package org.firstinspires.ftc.teamcode;

import android.provider.Settings;

import com.bylazar.field.PanelsField;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class AprilTagTesting extends OpMode {
    AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            1.25, 8.25, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    VisionPortal.Builder builder = new VisionPortal.Builder();

    Pose startPose;
    Pose currentPose;
//    Pose targetPose = new Pose(64.5, 135, Math.toRadians(135));
    Pose targetPose = new Pose(35, 60, Math.toRadians(135));
//    Pose targetPose = new Pose(0, 30, Math.toRadians(0));

    Follower follower;

    @Override
    public void init() {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
//        builder.setAutoStopLiveView(false);

        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(aprilTag, false);

        follower = Constants.createFollower(hardwareMap);
        // set a safe default starting pose so currentPose is never null
        Pose defaultPose = new Pose();
        follower.setStartingPose(defaultPose);
        follower.setPose(defaultPose);
        startPose = defaultPose;
        currentPose = defaultPose;

        telemetry = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
    }

    @Override
    public void start() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        }
    }

    @Override
    public void loop() {
        // Update currentPose only when we have detections; otherwise keep last-known pose
        int detectionCount = 0;
        if (aprilTag != null && aprilTag.getDetections() != null) {
            detectionCount = aprilTag.getDetections().size();
            if (detectionCount > 0) {
                try {
                    currentPose = new Pose(
                            72 + aprilTag.getDetections().get(0).robotPose.getPosition().x,
                            72 + aprilTag.getDetections().get(0).robotPose.getPosition().y,
                            aprilTag.getDetections().get(0).robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                    );
//                    currentPose = new Pose(
//                            aprilTag.getDetections().get(0).ftcPose.x,
//                            aprilTag.getDetections().get(0).ftcPose.y,
//                            aprilTag.getDetections().get(0).ftcPose.yaw);
                } catch (Exception e) {
                    // if something unexpected happens reading the detection, keep the previous pose
                    telemetry.addData("AprilTag", "detection read error: %s", e.getMessage());
                }
            }
        }

        // Ensure follower exists before using it
        if (follower == null) return;

        // Give the follower the latest known pose before updating its internal controllers
        if (currentPose != null) {
            follower.setPose(currentPose);
        }

        follower.update();

        if (!follower.isBusy()) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), targetPose))
                            .setLinearHeadingInterpolation(follower.getHeading(), targetPose.getHeading())
                            .build()
            );
        }

        // Provide telemetry to aid debugging on robot
        telemetry.addData("Detections", detectionCount);
        telemetry.addData("Pose", currentPose);
        telemetry.addData("FollowerBusy", follower.isBusy());
        telemetry.update();

        PanelsField.INSTANCE.getField().moveCursor(currentPose.getX(), currentPose.getY());
        PanelsField.INSTANCE.getField().update();
    }

    @Override
    public void stop() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }
}
