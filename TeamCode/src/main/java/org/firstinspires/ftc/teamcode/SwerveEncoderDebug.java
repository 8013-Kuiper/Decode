package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Code.SwerveModule;

// ══════════════════════════════════════════════════════════════════════════════
// ENCODER CALIBRATION HELPER — run this first to find your OFFSET values
// ══════════════════════════════════════════════════════════════════════════════
// Uncomment, run it in TeleOp, physically point every pod forward,
// then read the displayed angles and paste them into the OFFSET constants above.
//
@TeleOp
public class SwerveEncoderDebug extends OpMode {

    private  SwerveModule fl, fr, bl, br;

    @Override
    public void init() {
        fl  = new SwerveModule(hardwareMap,
                "frontLeft",  "frontLeftS",  "frontLeftA",  false, 0);
        fr = new SwerveModule(hardwareMap,
                "frontRight", "frontRightS", "frontRightA", true,  0);
        bl   = new SwerveModule(hardwareMap,
                "backLeft",   "backLeftS",   "backLeftA",   false, 0);
        br  = new SwerveModule(hardwareMap,
                "backRight",  "backRightS",  "backRightA",  true,  0);
    }

    @Override
    public void loop() {
        // Servos hold still; read raw angles
        telemetry.addLine("Point all pods STRAIGHT FORWARD, then record the angles below.");
        telemetry.addLine("Paste them into OFFSET_FL/FR/BL/BR in SwerveDriveTeleOp.");
        telemetry.addLine();
        telemetry.addData("FL raw angle (V)", "%.3f V  →  %.1f°", fl.getRawEncoderVoltage(), fl.getCurrentAngle());
        telemetry.addData("FR raw angle (V)", "%.3f V  →  %.1f°", fr.getRawEncoderVoltage(), fr.getCurrentAngle());
        telemetry.addData("BL raw angle (V)", "%.3f V  →  %.1f°", bl.getRawEncoderVoltage(), bl.getCurrentAngle());
        telemetry.addData("BR raw angle (V)", "%.3f V  →  %.1f°", br.getRawEncoderVoltage(), br.getCurrentAngle());
        telemetry.update();
    }

    @Override
    public void stop() {}
}
