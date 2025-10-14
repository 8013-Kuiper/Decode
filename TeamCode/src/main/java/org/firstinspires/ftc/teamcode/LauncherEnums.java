package org.firstinspires.ftc.teamcode;

public class LauncherEnums {
    private double speedTick = .1;
    public LauncherEnums(){

    }
    public enum States{
        MAX,
        OFF
    }
    public enum speedStates{
        SPEEDUP,
        SPEEDDOWN
    }


    public void setState(States state){

    }

    public void speed(speedStates state){
        if (state == speedStates.SPEEDDOWN){

        }
    }
}
