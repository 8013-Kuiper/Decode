package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class FinalTeleOp extends OpMode {
    //Turret constants
    final double driveTeeth = 59;
    final double drivenTeeth = 170;
    final double TicksPerDeg = 537.7*(drivenTeeth/driveTeeth)/360;

    //TODO: Tune these values
    final int turretMinDeg = -375;
    final int turretMaxDeg = 180;

    //Follower variables
//    Pose currentPose = new Pose(72, 72, 0);
    Pose currentPose = Constants.currentPose;
    Follower follower;
    boolean detectBlue=true;
    Pose blueGoal = new Pose(6,135);
    Pose redGoal = new Pose(144-6, 135);

    //Accessory Objects
    Servo spindex;
    Servo gate;
    MotorEx launcher;
    DcMotor intake;
    Motor turretRotation;;

    //Turret variables
    int target;
    double angle;
    double angleCorrected;
    double offset = 0;
    int targetRPM;
    double distance;

    //Debounce variables
    boolean overrideFlywheel = false;
    boolean overrideTurret = true;
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

        //Accessory initialization
        turretRotation.setRunMode(Motor.RunMode.PositionControl);
        turretRotation.setInverted(true);
        turretRotation.setPositionCoefficient(-0.05);
        turretRotation.setFeedforwardCoefficients(0.15,0.15);
        turretRotation.resetEncoder();
        turretRotation.setPositionTolerance(5);

        spindex = hardwareMap.get(Servo.class, "rotate");
        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void start() {
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(20, 0, 0);
        launcher.setFeedforwardCoefficients(0.35, 0.5);

        spindex.setPosition(0.5);
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
        //REPLACE FOR LIMELIGHT
        Constants.currentPose = currentPose;

        //Turret Control
        if (detectBlue) {
            angle = Math.atan2(blueGoal.getY() - currentPose.getY(), currentPose.getX() - blueGoal.getX());
            distance = currentPose.distanceFrom(blueGoal);
        } else {
            angle = Math.atan2(redGoal.getY() - currentPose.getY(), currentPose.getX() - redGoal.getX());
            distance = currentPose.distanceFrom(redGoal);
        }

        target = (int) ((180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading()))*TicksPerDeg);

        angleCorrected = (180 - Math.toDegrees(angle) - Math.toDegrees(currentPose.getHeading()));
        turretRotation.setTargetPosition((int) (target + offset));

        if (!overrideTurret) {
            if (angleCorrected > turretMaxDeg) {
                offset -= 360 * TicksPerDeg;
                angleCorrected -= 360;
            } else if (angleCorrected < turretMinDeg) {
                offset += 360 * TicksPerDeg;
                angleCorrected += 360;
            } else {
                turretRotation.setTargetPosition(target);
            }

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
        if (distance < 50) {
            targetRPM = (int) (16 * distance + 3205);
        } else {
            targetRPM = (int) (16 * distance + 3155);
        }

        //Transfer Controls
        if (gamepad2.a) {
            spindex.setPosition(1.0);
        } else if (gamepad2.b) {
            spindex.setPosition(0);
        } else {
            spindex.setPosition(0.5);
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
    }
}
