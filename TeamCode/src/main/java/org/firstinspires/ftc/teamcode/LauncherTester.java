package org.firstinspires.ftc.teamcode;

public class LauncherTester extends DriveConstants{
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        LauncherEnums flyWheelFunc = new LauncherEnums(flyWheel);

        if (gamepad1.a)
            flyWheelFunc.set();

        if (gamepad1.b)
            flyWheelFunc.decreasePower();
        if (gamepad1.y)
            flyWheelFunc.increasePower();

        if (gamepad1.left_trigger > .1)
            flyWheelFunc.setState(LauncherEnums.States.OFF);
        if (gamepad1.right_trigger > .1)
            flyWheelFunc.setState(LauncherEnums.States.MAX);

        telemetry.addData("Power to set:", flyWheelFunc.getPowerSet());
        telemetry.addData("Current power:", flyWheel.getPower());
        telemetry.addData("Velocity: ", flyWheel.getVelocity());

    }
}
