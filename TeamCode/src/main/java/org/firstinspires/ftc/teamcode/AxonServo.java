package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class AxonServo {
    private CRServo CRservo;
    private AnalogInput analog;
    private String name;
    public AxonServo(String Name, CRServo Servo, AnalogInput analog) {
        this.CRservo = Servo;
        this.analog = analog;
        this.name = Name;
    }
    //Variable declarations
    private static final double MAX_VOLTAGE = 3.273;
    private double SERVO_RPM = 86.96;
    private double ERROR_THRESHOLD = 80.0;
    private double CORRECTION_COEFFICIENT = 0.008;
    double savedRollover;
    boolean saveRollover;
    double lastVoltage = 0;
    double rollover = 0;
    double continuousVoltage = 0;
    double servoAngle = 0;
    boolean isBusy = false;

    //Initializer methods
    //These handle reading off of disk and allow the setting of attributes
    public void initializeServo(boolean SaveRollover, double Threshold, double Correction, boolean PowerModule) { //It is recommended that if you intend to operate the servo beyond one rotation and keeping track of it matters that you enable saving to the disk
        if (SaveRollover) {
            readLogFile();
            saveRollover = true;
        } else {
            saveRollover = false;
        }

        if (PowerModule) {
            SERVO_RPM = 86.96;
        } else {
            SERVO_RPM = 71.43;
        }
        ERROR_THRESHOLD = Threshold;
        CORRECTION_COEFFICIENT = Correction;
        lastVoltage = analog.getVoltage();
    }
    public void initializeServo(boolean SaveRollover) { //Running this method will keep the default parameters
        if (SaveRollover) {
            readLogFile();
            saveRollover = true;
        } else {
            saveRollover = false;
        }
        lastVoltage = analog.getVoltage();
    }

    //Getters & metering methods...
    public double findAngle() {
        servoAngle = (getContinuousVoltage() / MAX_VOLTAGE) * 360.0;
        return servoAngle;
    }
    public double getAngle() {
        return servoAngle;
    }
    public double getVoltage() {
        return analog.getVoltage();
    }
    public boolean isBusy() {
        return isBusy;
    }
    //End Getters & metering methods

    //File Saving/Access Methods
    //These methods handle the saving of the rollover value to disk, so that in the event of a power failure or the robot being turned off, the robot will remember where the servos are to zero them
    private File getFile() {
        return AppUtil.getInstance().getSettingsFile(name + "_rollover.txt");
    }
    public double loadRollover() {
        try {
            Scanner scanner = new Scanner(getFile());
            double value = scanner.nextDouble();
            scanner.close();

            //this exists because if the pod was jostled a little bit when powered off it could cause it to roll over
            if (value == -1 && getVoltage() < 0.2) {
                value = 0;
            } else if (value == 0 && getVoltage() > 3.2){
                value = -1;
            }
            return value;
        } catch (Exception e) {
            return 0; //default if file missing
        }
    }
    public void saveRollover(double rollover) {
        try {
            FileWriter writer = new FileWriter(getFile()); // overwrites
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
    //End File Saving Code

    //This method handles the voltage wraparound, which keeps track of the number of times the servo's analog output has 'rolled over' from 0V-3.3V or 3.3V-0V
    public double getContinuousVoltage() {
        double current = getVoltage();
        double delta = current - lastVoltage;
        double oldRollover = rollover;
        //Handle rollover
        if (delta > MAX_VOLTAGE / 2) rollover -=  1;
        if (delta < -MAX_VOLTAGE / 2) rollover += 1;
        continuousVoltage = current + (MAX_VOLTAGE * rollover);

        //This statement determines if the current rollover should be saved
        if (saveRollover) {
            if (rollover > 0 || rollover < -1) {
                //We don't want to have too many writes to disk, as not only can it wear down the disk but also slow the program
                if (rollover != oldRollover) {
                    saveRollover(rollover);
                }
            } else if (savedRollover != 0 && savedRollover != -1) {
                //0 and -1 are identical to loadRollover
                saveRollover(0.0);
            }
        }

        lastVoltage = current;
        return continuousVoltage;
    }

    //Servo control methods...
    public void runToPosition(double angle) { //Accepts inputs of any angle, and the servo will move to that position at max speed
        double error = angle - findAngle();

        double power;
        if (Math.abs(error) > ERROR_THRESHOLD) {
            isBusy = true;
            power = -Math.signum(error); //When power is positive axon servo rotates counterclockwise
        } else if (Math.abs(error) > 2){
            isBusy = true;
            power = -CORRECTION_COEFFICIENT * error;
        } else {
            isBusy = false;
            power = 0.0;
        }
        CRservo.setPower(power);
    }
    public void runToPosition(double angle, double speed) { //Accepts inputs of any angle, and the servo will move to that position at set speed
        double error = angle - findAngle();

        double power;
        if (Math.abs(error) > ERROR_THRESHOLD) {
            isBusy = true;
            power = -Math.signum(error);
        } else if (Math.abs(error) > 2){
            isBusy = true;
            power = -CORRECTION_COEFFICIENT * error;
        } else {
            isBusy = false;
            power = 0.0;
        }
        CRservo.setPower(power * speed);
    }
    public void setPower(double power) {
        CRservo.setPower(power);
    }
    public void setRPM(double RPM) {
        double power = RPM / SERVO_RPM;
        power = Math.min(power, 1);
        power = Math.max(power, -1);
        CRservo.setPower(power);
    }
    //End servo control methods
}