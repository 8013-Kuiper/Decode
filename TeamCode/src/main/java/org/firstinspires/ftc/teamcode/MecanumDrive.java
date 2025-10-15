package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class MecanumDrive {
    // Defining Motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // Defining Odometry Pods
    public DcMotorEx leftPod;
    public DcMotorEx rightPod;
    public DcMotorEx lateralPod;

    public Pose2D pose;
    private volatile boolean running = true;
    private static final double trackWidth = 14.5; // Distance between left and right pods in inches

    public MecanumDrive (HardwareMap hardwareMap, Pose2D pose) {
        this.pose = pose;

        frontLeft = hardwareMap.get(DcMotorEx.class, "");
        frontRight = hardwareMap.get(DcMotorEx.class, "");
        backLeft = hardwareMap.get(DcMotorEx.class, "");
        backRight = hardwareMap.get(DcMotorEx.class, "");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPod = hardwareMap.get(DcMotorEx.class, "leftPod");
        rightPod = hardwareMap.get(DcMotorEx.class, "rightPod");
        lateralPod = hardwareMap.get(DcMotorEx.class, "lateralPod");

        leftPod.setDirection(DcMotorSimple.Direction.FORWARD);
        rightPod.setDirection(DcMotorSimple.Direction.FORWARD);
        lateralPod.setDirection(DcMotorSimple.Direction.FORWARD);

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lateralPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lateralPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        startLocalizer();
    }

    /*
    Localize method to update the robot's position based on odometry pod readings
     */
    private void localize () {
        double prevLeft = 0;
        double prevRight = 0;
        double prevLateral = 0;

        double phi = (leftPod.getCurrentPosition() - prevLeft - rightPod.getCurrentPosition() + prevRight) / trackWidth;
        double delta_middle = (leftPod.getCurrentPosition() - prevLeft + rightPod.getCurrentPosition() - prevRight) / 2.0;
        double delta_lateral = lateralPod.getCurrentPosition() - prevLateral;

        double delta_x = delta_middle * Math.cos(pose.getHeading(AngleUnit.RADIANS)) - delta_lateral * Math.sin(pose.getHeading(AngleUnit.RADIANS));
        double delta_y = delta_middle * Math.sin(pose.getHeading(AngleUnit.RADIANS)) + delta_lateral * Math.cos(pose.getHeading(AngleUnit.RADIANS));
        pose = new Pose2D(DistanceUnit.INCH,pose.getX(DistanceUnit.INCH) + delta_x, pose.getY(DistanceUnit.INCH) + delta_y,AngleUnit.DEGREES, pose.getHeading(AngleUnit.DEGREES) + phi);

        prevLeft = leftPod.getCurrentPosition();
        prevRight = rightPod.getCurrentPosition();
        prevLateral = lateralPod.getCurrentPosition();
    }

    Thread localizerThread = new Thread(() -> {
        while (running) {
            localize();
            try {
                Thread.sleep(10); // ~100 Hz update rate
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    });

    private void startLocalizer() {
        localizerThread.start();
    }

    public void setPose(double x, double y, double heading) {
        pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, heading);
    }
}
