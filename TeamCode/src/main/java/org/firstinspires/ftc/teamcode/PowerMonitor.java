package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import java.util.ArrayList;

public class PowerMonitor {
    private DcMotorEx[] motors;
    private VoltageSensor voltageSensor;
    private ElapsedTime runtime;

    private double oldTime = 0;
    private double initialPercent = 100;
    private double chargeUsed = 0; // mAh
    private double milliamperes = 0;
    private double batteryVoltage = 0;
    private double initialBatteryVoltage;
    private double internalResistance = INITIAL_INTERNAL_RESISTANCE;
    private double absoluteCharge = 100;
    private double averageInternalResistance;
    ArrayList <Double> internalResistances = new ArrayList<>();
    private int index = 0;
    private static final int SAMPLE_SIZE = 100;
    public static final double BATTERY_CAPACITY = 3000; // mAh
    public static final double INITIAL_INTERNAL_RESISTANCE = 0.1;
    public static final double CONTROL_HUB_CURRENT = 0.3;
    public static final double AXON_SERVO_CURRENT_NO_LOAD = 0.15;
    public static final double NOMINAL_VOLTAGE = 12.0;

    public PowerMonitor(VoltageSensor voltageSensor, ElapsedTime runtime, DcMotorEx... motors) {
        this.voltageSensor = voltageSensor;
        this.runtime = runtime;
        this.motors = motors;
        update();
        initialBatteryVoltage = Math.max(batteryVoltage + 0.3, NOMINAL_VOLTAGE);
        initialPercent = voltageSocPercent(batteryVoltage);
        internalResistances.add(INITIAL_INTERNAL_RESISTANCE);
    }
    private static double voltageSocPercent(double ocv) {
        double result;
        result = 100 / (1 + Math.pow(Math.E, (-2.5 * (ocv - 12.2)))) + 2;
        result = Math.min(result, 100);
        return result;
    }

    public void update() {
        double currentTime = runtime.milliseconds();
        double deltaTime = currentTime - oldTime;
        batteryVoltage = voltageSensor.getVoltage();

        //Sum motor currents
        milliamperes = 0;
        for (DcMotorEx motor : motors) {
            milliamperes += motor.getCurrent(CurrentUnit.MILLIAMPS);
        }
        //Add other component's current
        milliamperes += (CONTROL_HUB_CURRENT) * 1000;
        milliamperes += SwerveTeleOP.servosBusy * AXON_SERVO_CURRENT_NO_LOAD * 1000;

        // Convert to mAh
        chargeUsed += (milliamperes * deltaTime) / 3600000.0;

        //Set initial voltage
        if (getCurrent() > 1.2) {
            //Calculate internal resistance
            internalResistance = Math.max(initialBatteryVoltage - batteryVoltage, 0) / (getCurrent() - CONTROL_HUB_CURRENT);

            double sum = 0;
            double average;
            for (Double IR : internalResistances) {
                sum += IR;
            }
            average = sum / internalResistances.size();

            if (internalResistance > average) {
                if (internalResistances.size() < SAMPLE_SIZE) {
                    internalResistances.add(internalResistance);
                    averageInternalResistance = (sum + internalResistance) / (internalResistances.size());
                } else {
                    if (index >= SAMPLE_SIZE) {
                        index = 0;
                    }
                    internalResistances.set(index, internalResistance);
                    sum = 0;
                    for (Double IR : internalResistances) {
                        sum += IR;
                    }
                    averageInternalResistance = sum / internalResistances.size();
                    index ++;
                }
            }

            absoluteCharge = Math.min(100 - ((averageInternalResistance - INITIAL_INTERNAL_RESISTANCE) / (INITIAL_INTERNAL_RESISTANCE * 4)) * 100, 100);
        }
        if (getCurrent() < 0.5) {
            //initialBatteryVoltage = Math.max(batteryVoltage, NOMINAL_VOLTAGE);
            initialBatteryVoltage = batteryVoltage;
        }
        oldTime = currentTime;
    }

    public double getCurrent() {
        return milliamperes / 1000;
    }
    public double getVoltage() {
        return batteryVoltage;
    }
    public double getChargeUsed() {
        return chargeUsed;
    }
    //Returns the estimated percent remaining based off of charged used
    public int getPercentRemaining() {
        return (int) (initialPercent - chargeUsed / BATTERY_CAPACITY * 100);
    }
    //Returns the absolute battery percent remaining based off of internal resistance. Does not account for battery wear, for example an aged battery may start at 50%, when that is that battery's 100%.
    public int getAbsolutePercent() {
        return (int) absoluteCharge;
    }
    public double getBaselineVoltage() {
        return initialBatteryVoltage;
    }
    public double getInternalResistance() {
        return internalResistance;
    }
    public double getAverageInternalResistance() {
        return averageInternalResistance;
    }
}