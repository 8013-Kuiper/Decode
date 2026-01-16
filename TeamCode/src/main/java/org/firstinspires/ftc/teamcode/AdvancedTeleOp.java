package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp (name="**TeleOp With April Tags**")
public class AdvancedTeleOp extends OpMode {
    // Drive Motors
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // Accessory Hardware
    Servo spindex;
    Servo gate;
    MotorEx launcher;
    DcMotor intake;
    DcMotor turretRotatation;
    int targetRPM;

    //Turret Min Max values
    int turretMinTicks = 0;
    int turretMaxTicks = 0;

    //Auto Turret Variables
    double kP = 0.1;
    double kI = 0;
    double kD = 0;
    double targetCenterX;

    double integral = 0;
    double lastError = 0;
    ElapsedTime pidTimer = new ElapsedTime();

    double MAX_POWER = 0.25;

    //April Tag Variables
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    boolean detectRed = false;
    boolean overrideFlywheel = false;
    boolean overrideTurret = true;

    // Override Debounce Variables
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
        initAprilTag();

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        spindex = hardwareMap.get(Servo.class, "rotate");
        launcher = new MotorEx(hardwareMap, "launcher", 28, 6000);
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        turretRotatation = hardwareMap.get(DcMotor.class, "turret");

        turretRotatation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretRotatation.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRotatation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    public void start() {
        launcher.setRunMode(MotorEx.RunMode.VelocityControl);
        launcher.setVeloCoefficients(20, 0, 0);
        launcher.setFeedforwardCoefficients(0.35, 0.35);

        turretRotatation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spindex.setPosition(0.5);
        gate.setPosition(1);
    }

    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower = drive - strafe + rotate;
        double backRightPower = drive + strafe - rotate;

        if (!gamepad1.left_bumper) {
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
        } else {
            frontLeft.setPower(0.5*frontLeftPower);
            frontRight.setPower(0.5*frontRightPower);
            backLeft.setPower(0.5*backLeftPower);
            backRight.setPower(0.5*backRightPower);
        }

        if (gamepad2.dpad_right) {
            detectRed = true;
        } else if (gamepad2.dpad_left) {
            detectRed = false;
        }

        if (overrideFlywheel) {
            if (gamepad2.right_trigger > 0.1) {
                launcher.set(gamepad2.right_trigger);
            } else {
                launcher.set(0);
            }
        } else {
            launcher.setVelocity(gamepad2.right_trigger * (((double) targetRPM /60*28)+250));
        }

        if (overrideTurret) {
            turretRotatation.setPower(0.25 * gamepad2.left_stick_x);
        } else {
//            updateTurretPID(detectRed, turretRotatation);
        }

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

        if (gamepad2.left_trigger > 0.1) {
            intake.setPower(gamepad2.left_trigger);
        } else if (gamepad2.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        //Target Velocity Calculations
        double distance = getDistance(detectRed);
        if (distance < 50) {
            targetRPM = (int) (16*distance + 3205);
        } else {
            targetRPM = (int) (16 * distance + 3155);
        }

        //Override Controls
            //Override Turret
            if (gamepad2.dpad_up && !previousOTstate && OTDebounceComplete) {
//                overrideTurret = !overrideTurret;
                OTDebounceComplete = false;
                OTDebounceStartTime = System.currentTimeMillis();
            }

            if (!OTDebounceComplete && (System.currentTimeMillis() - OTDebounceStartTime) >= debounceDelay) {
                OTDebounceComplete = true;
            }

            previousOTstate = gamepad2.dpad_up;

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

        //Encoder Reset Protection
        if (gamepad2.start && gamepad1.start && gamepad1.a && gamepad2.y){
            turretRotatation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretRotatation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Telemetry
        telemetry.addData("Distance", distance);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", launcher.getVelocity()/28*60);
        telemetry.addData("Override Auto Turret Controls", overrideTurret);
        telemetry.addData("Override Auto Flywheel Controls", overrideFlywheel);
        telemetry.addData("Detecting Red?", detectRed);
        telemetry.addData("Turret Ticks", turretRotatation.getCurrentPosition());
        telemetry.update();
    }

//--------------------------------------------------------------------------------------
//  Non OpMode Methods
//--------------------------------------------------------------------------------------
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
                    return det.ftcPose.range;
                }
            }
        } else {
            for (AprilTagDetection det : detections) {
                if (det.id == 20) {
                    return det.ftcPose.range;
                }
            }
        }
        return -1.0;
    }

    private double getCenterX(boolean isRed) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (isRed) {
            for (AprilTagDetection det : detections) {
                if (det.id == 24) {
                    return det.center.x;
                }
            }
        } else {
            for (AprilTagDetection det : detections) {
                if (det.id == 20) {
                    return det.center.x;
                }
            }
        }
        return 0;
    }

    public void updateTurretPID(boolean isRed, DcMotor turret) {
        double currentCenterX = getCenterX(isRed);
        double error = targetCenterX - currentCenterX;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        // PID calculations
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double pidOutput = kP * error + kI * integral + kD * derivative;

        // -----------------------------------------
        // Determine desired turret direction
        // -----------------------------------------

        double desiredPower = pidOutput;

        // Enforce max/min absolute power
        if (desiredPower > MAX_POWER) desiredPower = MAX_POWER;
        if (desiredPower < -MAX_POWER) desiredPower = -MAX_POWER;

        int currentPos = turret.getCurrentPosition();

        // -------------------------------------------------------------
        // Limit protection:
        // If desired motion would cross limits, rotate opposite instead
        // -------------------------------------------------------------
        if (desiredPower > 0 && currentPos >= turretMaxTicks) {
            // trying to go beyond max -> wrap opposite
            desiredPower = -MAX_POWER;
            integral = 0;  // prevent integral windup
        }

        if (desiredPower < 0 && currentPos <= turretMinTicks) {
            // trying to go beyond min -> wrap opposite
            desiredPower = MAX_POWER;
            integral = 0;
        }

        turret.setPower(desiredPower);
    }
}
