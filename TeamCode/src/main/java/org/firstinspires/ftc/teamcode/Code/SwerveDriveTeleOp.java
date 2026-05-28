package org.firstinspires.ftc.teamcode.Code;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * SwerveDriveTeleOp — 4-pod coaxial swerve drive for the REV Control Hub.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  HARDWARE CONFIG  (names must match your Driver Station configuration)
 * ═══════════════════════════════════════════════════════════════════════════
 *  DC Motors  (drive)          │  CRServos  (Axon MAX Mk2 steer)
 *  ────────────────────────    │  ──────────────────────────────
 *  "frontLeftDrive"            │  "frontLeftSteer"
 *  "frontRightDrive"           │  "frontRightSteer"
 *  "backLeftDrive"             │  "backLeftSteer"
 *  "backRightDrive"            │  "backRightSteer"
 *
 *  Analog Inputs (Axon MAX Mk2 absolute encoders — 5-pin Analog port)
 *  ─────────────────────────────────────────────────────────────────
 *  "frontLeftEncoder"
 *  "frontRightEncoder"
 *  "backLeftEncoder"
 *  "backRightEncoder"
 *
 *  IMU (built-in Control Hub gyro — no extra config needed)
 *  "imu"
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  ENCODER ZERO OFFSETS
 * ═══════════════════════════════════════════════════════════════════════════
 *  Run the "SwerveEncoderDebug" OpMode (below), manually point every pod
 *  straight forward, and record the getCurrentAngle() reading.  Enter each
 *  value in the OFFSET constants below.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 *  GAMEPAD 1 CONTROLS
 * ═══════════════════════════════════════════════════════════════════════════
 *  Left  stick  X/Y   → strafe / forward (translation)
 *  Right stick  X     → rotation
 *  Y button           → toggle Robot-Centric ↔ Field-Centric drive
 *  Left  bumper (hold)→ slow mode (33% speed)
 *  Right bumper (hold)→ turbo mode (100% speed, bypasses normal 80% cap)
 * ═══════════════════════════════════════════════════════════════════════════
 */
@Disabled
@TeleOp(name = "Swerve Drive TeleOp", group = "Drive")
public class SwerveDriveTeleOp extends OpMode {

    // ── Robot geometry ────────────────────────────────────────────────────────
    // Measure center-to-center between front & back wheels (same unit as you like).
    // Only the ratio L/R and W/R matter, so exact units are irrelevant.
    private static final double WHEEL_BASE   = 10.0;  // front ↔ rear distance
    private static final double TRACK_WIDTH  = 10.0;  // left ↔ right distance
    private static final double RADIUS       =
            Math.sqrt(WHEEL_BASE * WHEEL_BASE + TRACK_WIDTH * TRACK_WIDTH);

    // ── Encoder zero offsets (degrees) ───────────────────────────────────────
    // Set each to the raw encoder angle when the pod faces robot-forward.
    private static final double OFFSET_FL = 28.8;
    private static final double OFFSET_FR = 304.6;
    private static final double OFFSET_BL = 9.0;
    private static final double OFFSET_BR = 71.7;

    // ── Speed limits ─────────────────────────────────────────────────────────
    private static final double NORMAL_SPEED_SCALE = 0.80;
    private static final double SLOW_SPEED_SCALE   = 0.33;
    private static final double TURBO_SPEED_SCALE  = 1.00;

    // ── Input deadband ────────────────────────────────────────────────────────
    private static final double STICK_DEADBAND = 0.06;

    // ── Modules ───────────────────────────────────────────────────────────────
    private SwerveModule frontLeft, frontRight, backLeft, backRight;

    // ── IMU ───────────────────────────────────────────────────────────────────
    private IMU imu;

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean fieldCentric  = false;
    private boolean prevBtnY      = false;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ══════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {

        // ── Instantiate modules ───────────────────────────────────────────────
        //   SwerveModule(hardwareMap, driveMotor, steerServo, encoder, reverseMotor, offsetDeg)
        //   Flip reverseMotor so that positive drive power = forward on every pod.
        frontLeft  = new SwerveModule(hardwareMap,
                "frontLeft",  "frontLeftS",  "frontLeftA",  true, OFFSET_FL);
        frontRight = new SwerveModule(hardwareMap,
                "frontRight", "frontRightS", "frontRightA", false,  OFFSET_FR);
        backLeft   = new SwerveModule(hardwareMap,
                "backLeft",   "backLeftS",   "backLeftA",   true, OFFSET_BL);
        backRight  = new SwerveModule(hardwareMap,
                "backRight",  "backRightS",  "backRightA",  false,  OFFSET_BR);

        // ── IMU ───────────────────────────────────────────────────────────────
        // Adjust LogoFacingDirection / UsbFacingDirection to match how your
        // Control Hub is mounted inside the robot.
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);

        telemetry.addData("► Status", "Initialized — ready to start");
        telemetry.addData("► Drive mode", "Robot-Centric (Y to toggle)");
        telemetry.update();
    }

    // ══════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {
        loopTimer.reset();

        // ── Gamepad inputs ────────────────────────────────────────────────────
        double rawStrafe   =  gamepad1.left_stick_x;
        double rawForward  = -gamepad1.left_stick_y;  // stick Y is inverted
        double rawRotation =  gamepad1.right_stick_x;

        // Apply deadband with linear rescaling so 0 output starts at deadband edge
        double strafe   = applyDeadband(rawStrafe,   STICK_DEADBAND);
        double forward  = applyDeadband(rawForward,  STICK_DEADBAND);
        double rotation = applyDeadband(rawRotation, STICK_DEADBAND);

        // ── Speed scaling ─────────────────────────────────────────────────────
        double speedScale = NORMAL_SPEED_SCALE;
        if      (gamepad1.left_bumper)  speedScale = SLOW_SPEED_SCALE;
        else if (gamepad1.right_bumper) speedScale = TURBO_SPEED_SCALE;

        strafe   *= speedScale;
        forward  *= speedScale;
        rotation *= speedScale;

        // ── Field-centric toggle ──────────────────────────────────────────────
        boolean currBtnY = gamepad1.y;
        if (currBtnY && !prevBtnY) {
            fieldCentric = !fieldCentric;
            if (fieldCentric) imu.resetYaw(); // treat current heading as 0°
        }
        prevBtnY = currBtnY;

        // ── Field-centric rotation of translation vector ──────────────────────
        if (fieldCentric) {
            double heading = imu.getRobotYawPitchRollAngles()
                               .getYaw(AngleUnit.RADIANS);
            // Rotate the driver's input by -heading to align with the field
            double cosH  = Math.cos(-heading);
            double sinH  = Math.sin(-heading);
            double temp  = forward * cosH - strafe * sinH;
            strafe       = forward * sinH + strafe * cosH;
            forward      = temp;
        }

        // ── Swerve kinematics ─────────────────────────────────────────────────
        //
        //   Using the standard 4-wheel swerve decomposition:
        //
        //   a = strafe   - ω · (WHEEL_BASE  / R)
        //   b = strafe   + ω · (WHEEL_BASE  / R)
        //   c = forward  - ω · (TRACK_WIDTH / R)
        //   d = forward  + ω · (TRACK_WIDTH / R)
        //
        //   speed = hypot(component_x, component_y)
        //   angle = atan2(component_x, component_y)  [0° = robot-forward]
        //
        double halfL = WHEEL_BASE  / RADIUS;
        double halfW = TRACK_WIDTH / RADIUS;

        double a = strafe  - rotation * halfL;
        double b = strafe  + rotation * halfL;
        double c = forward - rotation * halfW;
        double d = forward + rotation * halfW;

        double speedFL = Math.hypot(b, d);
        double speedFR = Math.hypot(b, c);
        double speedBL = Math.hypot(a, d);
        double speedBR = Math.hypot(a, c);

        double angleFL = Math.toDegrees(Math.atan2(b, d));
        double angleFR = Math.toDegrees(Math.atan2(b, c));
        double angleBL = Math.toDegrees(Math.atan2(a, d));
        double angleBR = Math.toDegrees(Math.atan2(a, c));

        // Normalize all speeds so the max is ≤ 1.0
        double maxSpeed = Math.max(
                Math.max(speedFL, speedFR),
                Math.max(speedBL, speedBR));
        if (maxSpeed > 1.0) {
            speedFL /= maxSpeed;
            speedFR /= maxSpeed;
            speedBL /= maxSpeed;
            speedBR /= maxSpeed;
        }

        // ── Apply to modules ──────────────────────────────────────────────────
        boolean inputsNearZero =
                Math.hypot(strafe, forward) < 0.03 && Math.abs(rotation) < 0.03;

        if (inputsNearZero) {
            // Hold current steering angle; cut drive power
            frontLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
        } else {
            frontLeft.set (angleFL, speedFL);
            frontRight.set(angleFR, speedFR);
            backLeft.set  (angleBL, speedBL);
            backRight.set (angleBR, speedBR);
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        telemetry.addData("Drive Mode",
                fieldCentric ? "Field-Centric  [Y = toggle]"
                             : "Robot-Centric  [Y = toggle]");
        telemetry.addData("Speed Scale",
                speedScale == SLOW_SPEED_SCALE   ? "SLOW (LB)"  :
                speedScale == TURBO_SPEED_SCALE  ? "TURBO (RB)" : "Normal");
        telemetry.addLine();
        telemetry.addData("FL  angle | speed",
                "%.1f°  |  %.2f", frontLeft.getCurrentAngle(),  speedFL);
        telemetry.addData("FR  angle | speed",
                "%.1f°  |  %.2f", frontRight.getCurrentAngle(), speedFR);
        telemetry.addData("BL  angle | speed",
                "%.1f°  |  %.2f", backLeft.getCurrentAngle(),   speedBL);
        telemetry.addData("BR  angle | speed",
                "%.1f°  |  %.2f", backRight.getCurrentAngle(),  speedBR);
        if (fieldCentric) {
            double headingDeg = imu.getRobotYawPitchRollAngles()
                                   .getYaw(AngleUnit.DEGREES);
            telemetry.addData("Heading", "%.1f°", headingDeg);
        }
        telemetry.addData("Loop ms", "%.1f", loopTimer.milliseconds());
        telemetry.update();
    }

    // ══════════════════════════════════════════════════════════════════════════
    @Override
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /**
     * Applies a symmetric deadband with linear rescaling.
     * Input inside ±{@code band} returns 0.
     * Input at ±1.0 always returns ±1.0 regardless of band size.
     */
    private static double applyDeadband(double value, double band) {
        if (Math.abs(value) < band) return 0.0;
        return Math.signum(value) * ((Math.abs(value) - band) / (1.0 - band));
    }
}


