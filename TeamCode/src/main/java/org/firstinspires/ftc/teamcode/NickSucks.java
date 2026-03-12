package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
@TeleOp
public class NickSucks {
    //define variables
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    IMU imu;

    public void init(HardwareMap HwMap) {
        frontLeft = HwMap.get(DcMotor.class, "frontLeft");
        frontRight = HwMap.get(DcMotor.class, "frontRight");
        backLeft = HwMap.get(DcMotor.class, "backLeft");
        backRight = HwMap.get(DcMotor.class, "backRight");
        // green is same name a driver station

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = HwMap.get(IMU.class, "imu");
        //imu is for orientation of the robot in relation to the control hub

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(RevOrientation));
    }

    public void drive(double forward,double strafe,double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        //constrain the power

        maxPower = Math.max(maxPower,Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower,Math.abs(backLeftPower));
        maxPower = Math.max(maxPower,Math.abs(frontRightPower));
        maxPower = Math.max(maxPower,Math.abs(backRightPower));

        frontLeft.setPower(frontLeftPower/maxPower);
        backLeft.setPower(backLeftPower/maxPower);
        frontRight.setPower(frontRightPower/maxPower);
        backRight.setPower(backRightPower/maxPower);


    }


}




