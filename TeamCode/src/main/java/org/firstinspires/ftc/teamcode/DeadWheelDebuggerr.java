package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class DeadWheelDebuggerr extends LinearOpMode {
    DcMotor zero;
    DcMotor one;
    DcMotor two;
    DcMotor three;

    public void runOpMode () {
        zero = hardwareMap.get(DcMotor.class, "zero");
        one = hardwareMap.get(DcMotor.class, "one");
        two = hardwareMap.get(DcMotor.class, "two");
        three = hardwareMap.get(DcMotor.class, "three");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Zero", zero.getCurrentPosition());
            telemetry.addData("One", one.getCurrentPosition());
            telemetry.addData("Two", two.getCurrentPosition());
            telemetry.addData("Three", three.getCurrentPosition());
            telemetry.update();
        }
    }
}