package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;

    Servo gate;
    DcMotorEx launcher;
    DcMotor intake;

    boolean intakeAction = false;

    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        gate = hardwareMap.get(Servo.class, "gate");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");

        gate.setPosition(0);

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

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            if (gamepad1.a) {
                gate.setPosition(1.0);
            } else if (gamepad1.b) {
                gate.setPosition(-0.3);
            }

            if (gamepad1.right_trigger > 0.1) {
                launcher.setVelocity(6000 * gamepad1.right_trigger);
            } else {
                launcher.setVelocity(0);
            }

            if (gamepad1.dpad_down) {
                intakeAction=true;
            }

            if (intakeAction) {
                intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intake.setTargetPosition(500);
                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intake.setPower(0.25);
                if (!intake.isBusy()) {
                    intakeAction=false;
                }
            } else {
                if (gamepad1.left_trigger > 0.1) {
                    intake.setPower(1.0);
                } else if (gamepad1.left_bumper) {
                    intake.setPower(-1.0);
                } else {
                    intake.setPower(0.0);
                }
            }
        }
    }
}
