package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(group = "A", name = "SwerveTeleOP")

public class SwerveTeleOP extends LinearOpMode {

    //Declaring The Drivetrain Motors
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx rearLeft;
    DcMotorEx rearRight;

    //Declaring The Servo Motors
    CRServo frontLeftCR;
    CRServo frontRightCR;
    CRServo rearLeftCR;
    CRServo rearRightCR;

    //Declaring Analog Inputs
    AnalogInput frontLeftPos;
    AnalogInput frontRightPos;
    AnalogInput rearLeftPos;
    AnalogInput rearRightPos;

    //Declaring Sensors
    BNO055IMU IMU;

    //Declaring Variables
    double angle;
    double power;
    double rotation;

    //Control system variables
    public static double powerMultiplier;
    public static double powerSubtractor;
    public static boolean isNoInput;
    public static boolean resetPods;

    //Power Metering
    public static double servosBusy;
    VoltageSensor voltageSensor;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Instructions that run once upon initialization go here

        //Mapping the drivetrain motors to their ports
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        rearRight = hardwareMap.get(DcMotorEx.class, "backRight");

        //Setting the default direction for each motor (negative b/c direction mirrored)
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        //Setting the behavior for the motors. This way, the robot is harder to push
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Mapping the servos
        frontLeftCR = hardwareMap.get(CRServo.class, "frontLeftS");
        frontRightCR = hardwareMap.get(CRServo.class, "frontRightS");
        rearLeftCR = hardwareMap.get(CRServo.class, "backLeftS");
        rearRightCR = hardwareMap.get(CRServo.class, "backRightS");

        //Mapping the analog inputs
        frontLeftPos = hardwareMap.get(AnalogInput.class, "frontLeftA");
        frontRightPos = hardwareMap.get(AnalogInput.class, "frontRightA");
        rearLeftPos = hardwareMap.get(AnalogInput.class, "backLeftA");
        rearRightPos = hardwareMap.get(AnalogInput.class, "backRightA");

        //Voltage Sensor
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        PowerMonitor powerMonitor;

        //IMU
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        params.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(params);

        //Reset Encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Initialize Pods
        SwervePod fl = new SwervePod("frontLeft", frontLeftCR, frontLeft, frontLeftPos);
        SwervePod fr = new SwervePod("frontRight", frontRightCR, frontRight, frontRightPos);
        SwervePod rl = new SwervePod("backLeft", rearLeftCR, rearLeft, rearLeftPos);
        SwervePod rr = new SwervePod("backRight", rearRightCR, rearRight, rearRightPos);

        SwerveDrive drive = new SwerveDrive(fl, fr, rl, rr);

        SwervePod selectedPod = fl;
        String SelectedPod = "Front Left";

        //Initialize Controllers
        Controller driver = new Controller(gamepad1);
        Controller operator = new Controller(gamepad2);

        //Initialize Power Metering for All Motors
        powerMonitor = new PowerMonitor(voltageSensor, runtime,
                frontLeft, frontRight, rearLeft, rearRight
        );

        //Initialize Odometry
        //Odometry odometry = new Odometry(fl, fr, rl, rr, IMU, runtime, 0, 0);

        boolean initialized = false;
        double initRollover = 0;
        while (!isStarted()) {
            //Instructions to be repeated upon initialization until started

            if (!initialized) {
                fl.readLogFile();
                fr.readLogFile();
                rl.readLogFile();
                rr.readLogFile();
                initRollover = fl.savedRollover;
                initialized = true;
            }

            fl.updateVoltage();
            fr.updateVoltage();
            rl.updateVoltage();
            rr.updateVoltage();

            telemetry.addLine("------File Saving------");
            telemetry.addData("Rollover Saving", SwervePod.saveRollover);
            telemetry.addData("Save Count", fl.saveCount);
            telemetry.addData("Saved Rollover", fl.savedRollover);
            telemetry.addData("Initial Rollover", initRollover);
            telemetry.update();
        }

        while (isStarted() && !isStopRequested()) {
            //Things to be repeated when play pressed
            servosBusy = 0;
            if (fl.isBusy()) { servosBusy += 1; }
            if (fr.isBusy()) { servosBusy += 1; }
            if (rl.isBusy()) { servosBusy += 1; }
            if (rr.isBusy()) { servosBusy += 1; }

            powerMonitor.update();
            driver.update();
            operator.update();
            //odometry.update();

            fl.update();
            fr.update();
            rl.update();
            rr.update();

            powerSubtractor =  (1.0 - driver.getRightTrigger()) * 0.15;
            powerMultiplier = Math.min((1.0 - (driver.getLeftTrigger() - 0.30)), 1.0) - powerSubtractor;
            if (driver.getA()) {
                powerMultiplier /= 2.0;
            }

            angle = driver.getAngle();
            power = Math.pow(driver.getResultant(), 2) * powerMultiplier;
            rotation = Math.pow(driver.getRotation(), 3) * powerMultiplier;
            isNoInput = driver.isNoInput();

            if (driver.getLeftBumper()) {
                isNoInput = false;
                drive.xLock();
                drive.update();
            } else {
                drive.drive(angle, power, rotation);
                drive.update();
            }

            if (driver.getDpadRight() && driver.getX()) { //Zero pods at this position
                int val = 0;
                if (fl.getServoVoltage() > 1.6) {
                    val = -1;
                }
                fl.rollover = val;
                fr.rollover = val;
                rl.rollover = val;
                rr.rollover = val;
                fl.saveRollover(val);
                fr.saveRollover(val);
                rl.saveRollover(val);
                rr.saveRollover(val);

            }
            if (driver.getDpadUpPressed()) {
                selectedPod = fl;
                SelectedPod = "Front Left";
            } else if (driver.getDpadDownPressed()) {
                selectedPod = rr;
                SelectedPod = "Rear Right";
            } else if (driver.getDpadLeftPressed()) {
                selectedPod = rl;
                SelectedPod = "Rear Left";
            } else if (driver.getDpadRightPressed()) {
                selectedPod = fr;
                SelectedPod = "Rear Right";
            }
            if (driver.getYPressed()) {
                selectedPod.rollover += 1;
            } else if (driver.getXPressed()) {
                selectedPod.rollover -= 1;
            }

            SwervePod.saveRollover = !driver.getBToggle();
            resetPods = !driver.getRightBumper();

            //temporary data for telemetry and troubleshooting
            telemetry.addLine("-----Pod Direction-----");
            telemetry.addData("Selected Pod", SelectedPod);
            telemetry.addData("Voltage", (Math.round(fl.getServoVoltage() * 1000) / 1000.0 + " Volts"));
            telemetry.addData("Cont. Voltage", (Math.round(selectedPod.getContinuousVoltage() * 1000) / 1000.0 + " Volts"));
            telemetry.addData("Rollover Count", selectedPod.getRollover());
            telemetry.addData("Servo Position", (Math.round(selectedPod.getServoAngle() * 100) / 100.0 + "°"));
            telemetry.addData("Pod Position", (Math.round(selectedPod.getPodAngle() * 100) / 100.0 + "°"));
            telemetry.addLine();

            telemetry.addLine("--------Inputs--------");
            telemetry.addData("Angle", (Math.round(angle * 10) / 10.0 + "°"));
            telemetry.addData("Error", (Math.round(selectedPod.getError() * 100.0) / 100.0 + "°"));
            telemetry.addData("Direction", selectedPod.direction);
            telemetry.addData("Power", (Math.round(power * 100) / 100.0));
            telemetry.addData("Rotation", (Math.round(driver.getRotation() * 100) / 100.0));
            telemetry.addLine();

            telemetry.addLine("------File Saving------");
            telemetry.addData("Rollover Saving", SwervePod.saveRollover);
            telemetry.addData("Save Count", selectedPod.saveCount);
            telemetry.addData("Saved Rollover", selectedPod.savedRollover);
            telemetry.addLine();

            telemetry.addLine("------Odometry------");
            //telemetry.addData("Heading", (odometry.getHeading() + "°"));
            //telemetry.addData("Acceleration", (Math.round(odometry.getIMUAcceleration() * 1000.0) / 1000.0 + " ft/s^2"));
            //telemetry.addData("IMU Velocity", (Math.round(odometry.getIMUVelocity() * 1000.0) / 1000.0 + " ft/s"));
            //telemetry.addData("IMU Position (X, Y)", ((Math.round(odometry.getIMUPositionX() * 1000.0) / 1000.0 + " ft, ") + ((Math.round(odometry.getIMUPositionY() * 1000.0) / 1000.0 ) + " ft")));
            //telemetry.addData("Encoder Position (X, Y)", ((Math.round((odometry.getSDX() / 12.0) * 1000.0) / 1000.0 + " ft, ") + (Math.round((odometry.getSDY() / 12.0) * 1000.0) / 1000.0 + " ft")));
            telemetry.addLine();

            telemetry.addLine("------Power Metering------");
            telemetry.addData("Elapsed Time", (Math.round(runtime.seconds()) + " Seconds"));
            telemetry.addData("Battery Voltage", (Math.round(powerMonitor.getVoltage() * 1000) / 1000.0 + " Volts"));
            telemetry.addData("Current", (Math.round(powerMonitor.getCurrent() * 1000) / 1000.0) + " Amperes");
            telemetry.addData("Charge Used (mAh)", (Math.round(powerMonitor.getChargeUsed() * 1000) / 1000.0) + " mAh");
            telemetry.addData("Percent Remaining (Charge)", (powerMonitor.getPercentRemaining() + "%"));
            telemetry.addData("Battery Health (Resistance)", (powerMonitor.getAbsolutePercent() + "%"));
            telemetry.addData("Internal Resistance", (Math.round(powerMonitor.getInternalResistance() * 10000) / 10000.0 + "Ω"));
            telemetry.addData("Average Internal Resistance", (Math.round(powerMonitor.getAverageInternalResistance() * 10000) / 10000.0 + "Ω"));
            telemetry.addData("Baseline Voltage", (Math.round(powerMonitor.getBaselineVoltage() * 10000) / 10000.0 + " Volts"));

            telemetry.update();
        }
    }
}