package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class SwervePod {
    private CRServo turnServo;
    private DcMotorEx driveMotor;
    private AnalogInput analog;
    private String name;
    private int motorDirection;

    double servoTarget = 0;
    double servoAngle;
    double contVoltage;
    double savedRollover;
    double lastVoltage = 0;
    int saveCount = 0;
    double rollover = 0;
    double continuousVoltage = 0;
    double podRPM;
    int direction;
    boolean isBusy;
    double error;
    public static boolean saveRollover = true;
    private static final double MAX_VOLTAGE = 3.273;
    private static final double GEAR_RATIO = 56.0 / 24.0;
    private static final double DRIVE_GEAR_RATIO = 2.0;
    private static final double WHEEL_DIAMETER = 72.0; //millimeters
    private static final double ENCODER_RESOLUTION = ((((1.0 +(46.0 / 17.0))) * (1.0 + (46.0 / 17.0))) * 28.0); //ticks per revolution
    private static final double MAX_RPM = 435.0;
    private static final double SERVO_RPM = 86.96;
    private static final double ERROR_THRESHOLD = 120.0; //Servo angle threshold where power scales down linearly by CORRECTION_COEFFICIENT * error
    private static final double CORRECTION_COEFFICIENT = 0.005;
    public SwervePod(String name, CRServo turnServo, DcMotorEx driveMotor, AnalogInput analog) {
        this.turnServo = turnServo;
        this.driveMotor = driveMotor;
        this.analog = analog;
        this.name = name;
        if (driveMotor.getDirection().equals(DcMotor.Direction.FORWARD)) {
            this.motorDirection = 1;
        } else {
            this.motorDirection = -1;
        }
    }
    public void update() { //Run this every loop
        contVoltage = getContinuousVoltage();
        servoAngle = (contVoltage / MAX_VOLTAGE) * 360.0;
        if (!isBusy) {
            //updateOdometry();
        }
    }
    public double getContVoltage() {
        return contVoltage;
    }
    // Voltage to Servo Angle
    public double getServoAngle() {
        return servoAngle;
    }
    // Servo Angle to Pod Angle
    public double getPodAngle() {
        return getServoAngle() / GEAR_RATIO;
    }

    public double getServoVoltage() {
        return analog.getVoltage();
    }
    public void updateVoltage() {
        lastVoltage = getServoVoltage();
    }

    //File Saving/Access Methods
    //These methods handle the saving of the rollover value to disk, so that in the event of a power failure or the robot being turned off, the robot will remember where the pods are to zero them
    private File getFile() {
        return AppUtil.getInstance().getSettingsFile(name + "_rollover.txt");
    }
    public double loadRollover() {
        try {
            Scanner scanner = new Scanner(getFile());
            double value = scanner.nextDouble();
            scanner.close();

            //this exists because if the pod was jostled a little bit when powered off it could cause it to roll over.
            if (value == -1 && getServoVoltage() < 0.3) {
                value = 0;
            } else if (value == 0 && getServoVoltage() > 3.0){
                value = -1;
            }
            return value;
        } catch (Exception e) {
            return 0; //default if file missing
        }
    }
    public void saveRollover(double rollover) {
        try {
            FileWriter writer = new FileWriter(getFile()); //overwrites
            writer.write(String.valueOf(rollover));
            savedRollover = rollover;
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public void readLogFile() {
        rollover = loadRollover();
        savedRollover = rollover;
    }
    //End File Saving/Access Methods

    //This method handles the voltage wraparound, which keeps track of the number of times the servo's analog output has 'rolled over' from 0V-3.3V or 3.3V-0V
    public double getContinuousVoltage() {
        double current = getServoVoltage();
        double delta = current - lastVoltage;
        // Handle rollover
        if (delta > MAX_VOLTAGE / 2) rollover -= 1;
        if (delta < -MAX_VOLTAGE / 2) rollover += 1;
        continuousVoltage = current + (MAX_VOLTAGE * rollover);

        //This statement determines if the current rollover should be saved
        if (saveRollover) {
            //We don't want to have too many writes to disk, as not only can it wear down the disk but also slow the program.
            if (rollover != savedRollover) {
                if (!SwerveTeleOP.isNoInput) {
                    if (rollover > 0 || rollover < -1) {
                        saveRollover(rollover);
                        saveCount ++;
                    } else if (getServoVoltage() < 0.3 || getServoVoltage() > 3.0) {
                        if (savedRollover != 0 && savedRollover != -1) {
                            saveRollover(0);
                            saveCount ++;
                        }
                    } else {
                        saveRollover(rollover);
                        saveCount ++;
                    }
                } else if ((savedRollover != 0 && savedRollover != -1) && (rollover == 0 || rollover == -1)) {
                    if (getServoVoltage() < 0.3 || getServoVoltage() > 3.0) {
                        saveRollover(0);
                        saveCount ++;
                    } else {
                        saveRollover(rollover);
                        saveCount ++;
                    }
                }
            }
        }

        lastVoltage = current;
        return continuousVoltage;
    }

    public void setTargetAngle(double podAngle) {
        servoTarget = (podAngle * GEAR_RATIO);
    }

    //Troubleshooting Getters
    public double getRollover() {
        return rollover;
    }
    public double getError() {
        return error;
    }
    public boolean isBusy() {
        return isBusy;
    }
    //End Troubleshooting Getters

    //Begin Odometry Methods
    //End Odometry Methods

    //Begin Control Methods
    public void updateTurning() {
        double current = getServoAngle();
        current = current % (360 * GEAR_RATIO);  //normalize to one full pod rotation in degrees (360 * 25/12 = 750)
        error = servoTarget - current;

        //Handles situations where if the pod has to rotate over 180 degrees, it can go angle - 360 the other way (e.g. -90 instead of 270)
        if (Math.abs(error) > 200 * GEAR_RATIO) {
            if (error > 0) {
                error -= 360 * GEAR_RATIO;
            } else {
                error += 360 * GEAR_RATIO;
            }
        }

        //Handles situations where if the pod needs to rotate more than 90 degrees, it can rotate angle - 180 and reverse the motor speed (e.g. -45 instead of 135)
        direction = 1;
        if (Math.abs(error) > 95 * GEAR_RATIO) {
            if (error > 0) {
                error -= 180 * GEAR_RATIO;
            } else {
                error += 180 * GEAR_RATIO;
            }
            direction = direction * -1;
        }

        double power;
        if (Math.abs(error) > ERROR_THRESHOLD) {
            power = -Math.signum(error);
        } else if (Math.abs(error) > 2){
            power = -CORRECTION_COEFFICIENT * error;
        } else {
            power = 0.0;
        }

        if (Math.abs(error) > 30) {
            isBusy = true;
        } else {
            isBusy = false;
        }

        if (SwerveTeleOP.isNoInput && SwerveTeleOP.resetPods) {
            if (getContVoltage() > 1.3) {
                power = 1; //When power is positive axon servo rotates counterclockwise
            } else if (getContVoltage() < -1.3) {
                power = -1;
            } else {
                power = CORRECTION_COEFFICIENT * getServoAngle();
            }
        }

        turnServo.setPower(power);
        podRPM = power * (SERVO_RPM / GEAR_RATIO);
    }
    public void zeroPod(double power) {
        if (getContVoltage() > 1.3) {
            turnServo.setPower(power);
        } else if (getContVoltage() < -1.3) {
            turnServo.setPower(-power);
        } else {
            turnServo.setPower(CORRECTION_COEFFICIENT * getServoAngle());
        }
    }
    public void setDrivePower(double power) {
        if (isBusy) {
            driveMotor.setPower((podRPM / MAX_RPM) * -motorDirection);
        } else {
            driveMotor.setPower(power * direction + (podRPM / MAX_RPM) * -motorDirection);
        }
    }
}