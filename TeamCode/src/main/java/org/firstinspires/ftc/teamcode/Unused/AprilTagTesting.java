package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class AprilTagTesting extends OpMode {
    AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            1.25, 8.25, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    VisionPortal.Builder builder = new VisionPortal.Builder();

    Pose startPose;

    boolean go = true;
    Pose currentPose;
    Pose targetPose = new Pose(60, 80, Math.toRadians(135));
//    Pose targetPose = new Pose(35, 60, Math.toRadians(135));
//    Pose targetPose = new Pose(0, 30, Math.toRadians(0));

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

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
        try {
            sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        if (visionPortal != null && aprilTag != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
            go=false;
        }
    }

    private Pose getRobotPoseFromCamera(){
        // Update currentPose only when we have detections; otherwise keep last-known pose
        int detectionCount = 0;
        if (aprilTag != null && aprilTag.getDetections() != null) {
            detectionCount = aprilTag.getDetections().size();
            if (detectionCount > 0) {
                try {
                    currentPose = new Pose(
                            72 + aprilTag.getDetections().get(0).robotPose.getPosition().y,
                            Math.abs(aprilTag.getDetections().get(0).robotPose.getPosition().x - 72),
                            aprilTag.getDetections().get(0).robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                    );
                } catch (Exception e) {
                    // if something unexpected happens reading the detection, keep the previous pose
                    telemetry.addData("AprilTag", "detection read error: %s", e.getMessage());
                }
            }
        }

        return currentPose;
    }
}
