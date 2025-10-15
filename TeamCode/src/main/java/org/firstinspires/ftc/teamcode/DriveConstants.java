package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class DriveConstants extends LinearOpMode {

    public DcMotorEx flyWheel;

    public DcMotorEx frontLeftM;
    public Servo frontLeftS;

    public void initRobot(){

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

        frontLeftM = hardwareMap.get(DcMotorEx.class, "frontLeftM");


    }
}
