package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MecanumRobotCentricOpMode extends OpMode {

    MecanumRobotCentricOpMode drive = new MecanumRobotCentricOpMode();
    double forward, strafe, rotate;


    @Override
    public void init() {
        drive.init();
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
    }

}
