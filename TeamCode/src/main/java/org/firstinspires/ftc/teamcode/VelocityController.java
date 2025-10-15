package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class VelocityController extends LinearOpMode {
    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    gamepadEx gamepad1

    double RPM_Target = 5000;

    public void runOpMode(){
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                RPM_Target += 50;
            } else if (gamepad1.b) {
                RPM_Target -= 50;
            } else if (gamepad1.x) {
                RPM_Target = 0;
            }
            leftMotor.setVelocity(RPM_Target / 60 * 28);
            rightMotor.setVelocity(RPM_Target / 60 * 28);

            telemetry.addData("RPM", RPM_Target);
            telemetry.update();
        }
    }
}
