package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    Servo spindex;
    Servo gate;
    DcMotorEx launcher;
    DcMotor intake;
    DcMotor turretRotatation;

    boolean intakeAction = false;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        spindex = hardwareMap.get(Servo.class, "rotate");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");

        turretRotatation = hardwareMap.get(DcMotor.class, "turret");

        spindex.setPosition(0.5);
        gate.setPosition(1);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

//        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double frontLeftPower = drive + strafe + rotate;
            double frontRightPower = drive - strafe - rotate;
            double backLeftPower = drive - strafe + rotate;
            double backRightPower = drive + strafe - rotate;

            if (!gamepad1.right_bumper) {
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

            if (gamepad2.a) {
                spindex.setPosition(1.0);
            } else if (gamepad2.b) {
                spindex.setPosition(0);
            } else {
                spindex.setPosition(0.5);
            }

            if (gamepad2.right_trigger > 0.1) {
                launcher.setVelocity(6000 * gamepad2.right_trigger);
            } else {
                launcher.setVelocity(0);
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

            turretRotatation.setPower(0.25*gamepad2.left_stick_x);
        }
    }
}
