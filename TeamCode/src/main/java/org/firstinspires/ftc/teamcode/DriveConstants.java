package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class DriveConstants extends LinearOpMode {

    public DcMotorEx flyWheel;
    public void initRobot(){
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

    }
}
