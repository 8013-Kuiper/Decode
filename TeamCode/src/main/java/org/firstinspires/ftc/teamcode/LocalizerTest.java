package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class LocalizerTest extends LinearOpMode {
    DcMotor leftPod;
    DcMotor rightPod;
    DcMotor lateralPod;

    public void runOpMode () {
        leftPod = hardwareMap.get(DcMotor.class, "frontRight");
        rightPod = hardwareMap.get(DcMotor.class, "frontLeft");
        lateralPod = hardwareMap.get(DcMotor.class, "backLeft");

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPod.setDirection(DcMotorSimple.Direction.FORWARD);
        rightPod.setDirection(DcMotorSimple.Direction.FORWARD);
        lateralPod.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left Pod", leftPod.getCurrentPosition());
            telemetry.addData("Right Pod", rightPod.getCurrentPosition());
            telemetry.addData("Lateral Pod", lateralPod.getCurrentPosition());
            telemetry.update();
        }
    }

}
