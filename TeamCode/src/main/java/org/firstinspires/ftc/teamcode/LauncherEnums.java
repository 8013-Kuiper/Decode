package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LauncherEnums {
    private DcMotorEx flyWheel;
    private double PowerSet;
    private final double PowerTick = .1;

    public LauncherEnums(@NonNull DcMotorEx flyWheel){
        this.flyWheel = flyWheel;
    }

    public void increasePower(){
        PowerSet += PowerTick;
    }
    public void decreasePower(){
        PowerSet -= PowerTick;
    }

    public enum States{
        MAX,
        OFF
    }

    public void setState(States state){
        if (state == States.MAX)
            flyWheel.setPower(1);
        else
        if (state == States.OFF)
            flyWheel.setPower(0);

    }

    public void set(){
        flyWheel.setPower(PowerSet);
    }

    public double getPowerSet() {
        return PowerSet;
    }
}
