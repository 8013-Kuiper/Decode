package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//@Disabled
@TeleOp
public class FinalTeleOp extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    //Turret constants
    final double driveTeeth = 59;
    final double drivenTeeth = 170;
    final double TicksPerDeg = 537.7*(drivenTeeth/driveTeeth)/360;

    final int turretMinDeg = -225;
    final int turretMaxDeg = 350;

    //Follower variables
    Pose currentPose = new Pose(9.5, 9.5, 0);
//    Pose currentPose = Constants.currentPose;
    Follower follower;
    boolean detectBlue = true;
    Pose blueGoal = new Pose(6,135);
    Pose redGoal = new Pose(144-6, 135);

    //Accessory Objects
    DcMotor spin;
    Servo gate;
    MotorEx launcher;
    DcMotor intake;
    Motor turretRotation;;

    //Turret variables
    int target;
    double angle;
    double angleCorrected;
    int count = 0;
    int targetRPM = 3500;
    double distance;

    double offset = 0;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 10.25, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -35, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    TelemetryManager Telemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    //Debounce variables
    boolean overrideFlywheel = false;
    boolean overrideTurret = false;
    final long debounceDelay = 200;
        // Override Turret
        boolean previousOTstate = false;
        boolean OTDebounceComplete = true;
        long OTDebounceStartTime = 0;

        //Override FlyWheel
        boolean previousFWstate = false;
        boolean FWDebounceComplete = true;
        long FWDebounceStartTime = 0;

    public void init() {
        //Drive train initialization
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(currentPose);
        follower.update();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        //Limelight
//        limelight = hardwareMap.get(Limelight3A.class, "camera");
//        limelight.pipelineSwitch(0);

        VisionPortal.Builder builder = new VisionPortal.Builder();

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        //Accessory initialization
        turretRotation = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);
        turretRotation.setRunMode(Motor.RunMode.RawPower);
        turretRotation.setInverted(true);
//        turretRotation.setPositionCoefficient(-0.058);
//        turretRotation.setFeedforwardCoefficients(0.15,0.15);
//        turretRotation.resetEncoder();
//        turretRotation.setPositionTolerance(5);

        spin = hardwareMap.get(DcMotor.class, "rotate");
        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        launcher.setInverted(true);
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void start() {
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(10, 0.5, 0.5);
        launcher.setFeedforwardCoefficients(0.35, 0.5);

        spin.setPower(0);
        gate.setPosition(1);

        if (detectBlue) {
            angle = Math.atan2((currentPose.getX() - blueGoal.getX()), (currentPose.getY()) - blueGoal.getY());
        }

        follower.startTeleOpDrive();
    }

    public void loop() {
        //Drive Train Control
        follower.update();
        currentPose = follower.getPose();
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        Constants.currentPose = currentPose;

//        LLResult result = limelight.getLatestResult();
        double robotYaw = Math.toDegrees(currentPose.getHeading());
//        limelight.updateRobotOrientation(robotYaw);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 20 || detection.id ==24){
                double x = 72 + (detection.robotPose.getPosition().y);
                double y = 72 + (-detection.robotPose.getPosition().x);
                Telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
//                Telemetry.addData("Corrected MT2 Location:", "(" + (x + 72) + ", " + (y+72) + ")");
                currentPose = new Pose(x, y, detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS));
                follower.setPose(currentPose);
            }
        }   // end for() loop
        follower.update();

        //Turret Control
        if (detectBlue) {
            angle = Math.atan2(blueGoal.getY() - currentPose.getY(), currentPose.getX() - blueGoal.getX());
            distance = currentPose.distanceFrom(blueGoal);
        } else {
            angle = Math.atan2(redGoal.getY() - currentPose.getY(), currentPose.getX() - redGoal.getX());
            distance = currentPose.distanceFrom(redGoal);
        }

        target = (int) ((180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading()))*TicksPerDeg);

        angleCorrected = (180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading())) + count*360 + offset;

//        turretRotation.setRunMode(Motor.RunMode.PositionControl);
        if (angleCorrected > 350) {
            count --;
        } else if (angleCorrected < -225) {
            count ++;
        }

        Telemetry.addData("count", count);

//        target = (int) (angleCorrected*TicksPerDeg);
//
//        turretRotation.setTargetPosition(target);
//
//        turretRotation.set(0.05);
//
//        offset -= gamepad2.left_stick_x * 5;
        turretRotation.set(0.25*gamepad2.left_stick_x);

        //Side control
        if (gamepad2.dpad_right) {
            detectBlue = false;
        } else if (gamepad2.dpad_left) {
            detectBlue = true;
        }

        //Flywheel control
        if (overrideFlywheel) {
            if (gamepad2.right_trigger > 0.1) {
                launcher.set(gamepad2.right_trigger);
            } else {
                launcher.set(0);
            }
        } else {
            launcher.setVelocity(gamepad2.right_trigger * (((double) targetRPM /60*28)+225));
        }

//        if (gamepad2.dpad_up) {
//            targetRPM += 50;
//        } else if (gamepad2.dpad_down) {
//            targetRPM -= 50;
//        }

        targetRPM = (int) (10.6 * distance + 3100);

        //Target Velocity Calculation

        //Transfer Controls
        if (gamepad2.a) {
            spin.setPower(1.0);
        } else if (gamepad2.b) {
            spin.setPower(-1);
        } else {
            spin.setPower(0);
        }

        if (gamepad2.right_bumper){
            gate.setPosition(1);
        } else {
            gate.setPosition(0);
        }

        //Intake Controls
        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(gamepad2.left_trigger);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
//
//        if (gamepad1.a && gamepad2.a) {
//            turretRotation.resetEncoder();
//        }

        //Override Controls
            //Override Flywheel
            if (gamepad2.y && !previousFWstate && FWDebounceComplete) {
                overrideFlywheel = !overrideFlywheel;
                FWDebounceComplete = false;
                FWDebounceStartTime = System.currentTimeMillis();
            }

            if (!FWDebounceComplete && (System.currentTimeMillis() - FWDebounceStartTime) >= debounceDelay) {
                FWDebounceComplete = true;
            }

            previousFWstate = gamepad2.y;

            //Override Turret

        Telemetry.addData("current Pose", currentPose);
        Telemetry.addData("RPM", launcher.getVelocity()/28*60);
        Telemetry.addData("Override", angleCorrected);
        Telemetry.addData("target RPM", targetRPM);
//        Telemetry.addData("trigger", );
        Telemetry.update(telemetry);
    }
}
