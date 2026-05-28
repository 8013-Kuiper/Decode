package org.firstinspires.ftc.teamcode;

public class SwerveDrive {
    private SwervePod frontLeft;
    private SwervePod frontRight;
    private SwervePod rearLeft;
    private SwervePod rearRight;

    private static final double LENGTH = 17.4; //inches
    private static final double WIDTH = 17.4;
    public SwerveDrive(SwervePod fl, SwervePod fr, SwervePod rl, SwervePod rr) {
        this.frontLeft = fl;
        this.frontRight = fr;
        this.rearLeft = rl;
        this.rearRight = rr;
    }
    public void drive(double angle, double power, double rotation) {

        if (power < 0.01 && Math.abs(rotation) < 0.01) {
            //No input, don't update targets, let pods hold their last position
            frontLeft.setDrivePower(0);
            frontRight.setDrivePower(0);
            rearLeft.setDrivePower(0);
            rearRight.setDrivePower(0);
            return;
        }

        //Convert polar input to X/Y
        double vx = power * Math.cos(Math.toRadians(angle));
        double vy = power * Math.sin(Math.toRadians(angle));

        //Robot geometry
        double L = LENGTH / 2;
        double W = WIDTH / 2;
        double R = Math.sqrt(L * L + W * W);

        double rotFL =  1;
        double rotFR = -1;
        double rotRL = -1;
        double rotRR =  1;

        //Combine translation + rotation per wheel
        double fl_x = vx + rotFL * rotation * (L / R);
        double fl_y = vy + rotFL * rotation * (W / R);

        double fr_x = vx + rotFR * rotation * (L / R);
        double fr_y = vy - rotFR * rotation * (W / R);

        double rl_x = vx - rotRL * rotation * (L / R);
        double rl_y = vy + rotRL * rotation * (W / R);

        double rr_x = vx - rotRR * rotation * (L / R);
        double rr_y = vy - rotRR * rotation * (W / R);

        //Wheel speeds
        double fl_speed = Math.sqrt(fl_x * fl_x + fl_y * fl_y);
        double fr_speed = Math.sqrt(fr_x * fr_x + fr_y * fr_y);
        double rl_speed = Math.sqrt(rl_x * rl_x + rl_y * rl_y);
        double rr_speed = Math.sqrt(rr_x * rr_x + rr_y * rr_y);

        //Normalize speeds
        double max = Math.max(Math.max(fl_speed, fr_speed), Math.max(rl_speed, rr_speed));
        if (max > 1.0) {
            fl_speed /= max;
            fr_speed /= max;
            rl_speed /= max;
            rr_speed /= max;
        }

        //Wheel angles
        double fl_angle = Math.toDegrees(Math.atan2(fl_x, fl_y)) - 90;
        double fr_angle = Math.toDegrees(Math.atan2(fr_x, fr_y)) - 90;
        double rl_angle = Math.toDegrees(Math.atan2(rl_x, rl_y)) - 90;
        double rr_angle = Math.toDegrees(Math.atan2(rr_x, rr_y)) - 90;

        //Send to pods
        frontLeft.setTargetAngle(fl_angle);
        frontRight.setTargetAngle(fr_angle);
        rearLeft.setTargetAngle(rl_angle);
        rearRight.setTargetAngle(rr_angle);

        frontLeft.setDrivePower(fl_speed);
        frontRight.setDrivePower(fr_speed);
        rearLeft.setDrivePower(rl_speed);
        rearRight.setDrivePower(rr_speed);
    }
    public void xLock() {
        frontLeft.setTargetAngle(-45);
        frontRight.setTargetAngle(45);
        rearLeft.setTargetAngle(45);
        rearRight.setTargetAngle(-45);

        frontLeft.setDrivePower(0);
        frontRight.setDrivePower(0);
        rearLeft.setDrivePower(0);
        rearRight.setDrivePower(0);
    }

    public void update() {
        frontLeft.updateTurning();
        frontRight.updateTurning();
        rearLeft.updateTurning();
        rearRight.updateTurning();
    }
}