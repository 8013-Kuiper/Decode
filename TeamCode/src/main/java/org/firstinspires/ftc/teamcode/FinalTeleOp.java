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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//@Disabled
@TeleOp
public class FinalTeleOp extends OpMode {
    //Turret constants
    final double driveTeeth = 59;
    final double drivenTeeth = 170;
    final double TicksPerDeg = 537.7*(drivenTeeth/driveTeeth)/360;

    //TODO: Tune these values
    final int turretMinDeg = -225;
    final int turretMaxDeg = 350;

    //Follower variables
    Pose currentPose = new Pose(72, 72, 0);
//    Pose currentPose = Constants.currentPose;
    Follower follower;
    boolean detectBlue = true;
    Pose blueGoal = new Pose(6,135);
    Pose redGoal = new Pose(144-6, 135);

    //Accessory Objects
    Servo spin;
    Servo gate;
    MotorEx launcher;
    DcMotor intake;
    Motor turretRotation;;

    //Turret variables
    int target;
    double angle;
    double angleCorrected;
    int count = 0;
    int targetRPM;
    double distance;

    double offset = 0;

    Limelight3A limelight;

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

        //Limelight
        limelight = hardwareMap.get(Limelight3A.class, "camera");
        limelight.pipelineSwitch(0);

        //Accessory initialization
        turretRotation = new Motor(hardwareMap, "turret", Motor.GoBILDA.RPM_312);
        turretRotation.setRunMode(Motor.RunMode.PositionControl);
        turretRotation.setInverted(true);
        turretRotation.setPositionCoefficient(-0.05);
        turretRotation.setFeedforwardCoefficients(0.15,0.15);
        turretRotation.resetEncoder();
        turretRotation.setPositionTolerance(5);

        spin = hardwareMap.get(Servo.class, "rotate");
        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        launcher.setInverted(true);
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void start() {
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(20, 0, 0);
        launcher.setFeedforwardCoefficients(0.35, 0.5);

        spin.setPosition(0.5);
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
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true);

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
                currentPose = new Pose(x, y, botpose_mt2.getOrientation().getYaw(AngleUnit.RADIANS));
                follower.setPose(currentPose);
            } else {
                currentPose = follower.getPose();
            }
        } else {
            currentPose = follower.getPose();
        }

        //Turret Control
        if (detectBlue) {
            angle = Math.atan2(blueGoal.getY() - currentPose.getY(), currentPose.getX() - blueGoal.getX());
            distance = currentPose.distanceFrom(blueGoal);
        } else {
            angle = Math.atan2(redGoal.getY() - currentPose.getY(), currentPose.getX() - redGoal.getX());
            distance = currentPose.distanceFrom(redGoal);
        }

        target = (int) ((180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading()))*TicksPerDeg);

        angleCorrected = (180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading())) + count*360;

        if (!overrideTurret) {
            if (angleCorrected > 350) {
                count --;
            } else if (angleCorrected < -225) {
                count ++;
            }

            Telemetry.addData("count", count);

            target = (int) (angleCorrected*TicksPerDeg);

            turretRotation.setTargetPosition(target);

            turretRotation.set(0.05);
        } else {
            turretRotation.setRunMode(Motor.RunMode.RawPower);
            turretRotation.set(0.25*gamepad2.left_stick_x);
        }

        //Side control
        if (gamepad2.dpad_right) {
            detectBlue = true;
        } else if (gamepad2.dpad_left) {
            detectBlue = false;
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

        //Target Velocity Calculations
        targetRPM = (int) (10.6 * distance + 3100);

        //Transfer Controls
        if (gamepad2.a) {
            spin.setPosition(1.0);
        } else if (gamepad2.b) {
            spin.setPosition(0);
        } else {
            spin.setPosition(0.5);
        }

        if (gamepad2.right_bumper){
            gate.setPosition(0);
        } else {
            gate.setPosition(1);
        }

        //Intake Controls
        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(gamepad2.left_trigger);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        //Override Controls
            //Override Flywheel
            if (gamepad2.dpad_down && !previousFWstate && FWDebounceComplete) {
                overrideFlywheel = !overrideFlywheel;
                FWDebounceComplete = false;
                FWDebounceStartTime = System.currentTimeMillis();
            }

            if (!FWDebounceComplete && (System.currentTimeMillis() - FWDebounceStartTime) >= debounceDelay) {
                FWDebounceComplete = true;
            }

            previousFWstate = gamepad2.dpad_down;

            //Override Turret

            if (gamepad2.dpad_up && !previousOTstate && OTDebounceComplete) {
                overrideTurret = !overrideTurret;
                OTDebounceComplete = false;
                OTDebounceStartTime = System.currentTimeMillis();
            }

            if (!OTDebounceComplete && (System.currentTimeMillis() - OTDebounceStartTime) >= debounceDelay) {
                OTDebounceComplete = true;
            }

            previousOTstate = gamepad2.dpad_up;

        Telemetry.addData("current Pose", currentPose);
        Telemetry.addData("RPM", launcher.getVelocity()/28*60);
        Telemetry.addData("Override", overrideTurret);
        Telemetry.update(telemetry);
    }
}
