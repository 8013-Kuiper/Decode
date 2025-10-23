package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class SwervePod {
    private DcMotorEx motor;
    private ServoImplEx servo;
    private AnalogInput analogInput;
    public SwervePod(@NonNull DcMotorEx motor, @NonNull ServoImplEx servo, @NonNull AnalogInput analogInput){
        this.motor = motor;
        this.servo = servo;
        this.analogInput = analogInput;
    }

    public ServoImplEx getServo() {
        return servo;
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public double readAnalogInput(){

        return analogInput.getVoltage()/10;
    }
}
