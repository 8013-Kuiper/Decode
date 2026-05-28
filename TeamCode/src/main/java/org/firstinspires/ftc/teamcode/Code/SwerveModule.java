package org.firstinspires.ftc.teamcode.Code;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * SwerveModule — represents one coaxial swerve pod.
 *
 * Hardware per pod:
 *   - 1× Axon MAX Mk2 (configured as CRServo) for steering
 *   - 1× Axon MAX Mk2 absolute encoder (connected to an Analog Input port)
 *   - 1× DC motor for driving
 *
 * The Axon MAX Mk2 encoder outputs 0–3.3 V over a full 360° rotation.
 * It must be wired to one of the Control Hub's 5-pin Analog ports.
 *
 * Steering is controlled via a PID loop that runs every loop() call.
 * Module optimization: if the required turn is >90°, the drive direction
 * is flipped and the module turns the shorter way instead.
 */
public class SwerveModule {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final DcMotor     driveMotor;
    private final CRServo     steerServo;
    private final AnalogInput steerEncoder;

    // ── Steering PID ──────────────────────────────────────────────────────────
    // Tune kP first (increase until it oscillates, then back off ~30%).
    // Add kD to dampen overshoot. Keep kI small or 0; the absolute encoder
    // means there is no accumulating positional error between sessions.
    private static final double kP = 0.08;
    private static final double kI = 0.0;
    private static final double kD = 0.004;

    private double pidIntegral  = 0.0;
    private double pidLastError = 0.0;
    private long   pidLastTimeMs;

    // Anti-windup clamp on the integral term (degrees·s)
    private static final double INTEGRAL_CLAMP = 60.0;

    // Minimum drive power below which we coast (avoids motor chatter at zero)
    private static final double DRIVE_DEADBAND = 0.02;

    // ── Belt-drive ratio ──────────────────────────────────────────────────────
    // The 24-tooth pulley on the Axon drives a 56-tooth pulley on the pod.
    // One full servo revolution  →  (24 / 56) ≈ 154.3° of pod rotation.
    // One full pod revolution    →  (56 / 24) ≈ 2.333 servo revolutions.
    //
    // The Axon's absolute encoder only sees 0–360° of the *servo* shaft, so it
    // wraps around ~2.33 times per full pod turn.  We convert to pod-space by
    // multiplying continuous servo degrees by BELT_RATIO.
    private static final double BELT_RATIO = 24.0 / 56.0; // ≈ 0.4286

    // ── Encoder wrap tracking ─────────────────────────────────────────────────
    // Because the encoder resets to 0° every servo revolution, we maintain a
    // "continuous" servo angle by detecting and accumulating wrap events.
    // A jump of >180° in the raw reading means the encoder crossed the 0/360
    // boundary; we adjust continuousEncoderDeg accordingly.
    private double lastRawEncoderDeg;
    private double continuousEncoderDeg;

    // ── Encoder calibration ───────────────────────────────────────────────────
    // Adjust ENCODER_OFFSET_DEG per module so that 0° = forward on each pod.
    // Measure getCurrentAngle() when the pod is pointing forward, then set
    // ENCODER_OFFSET_DEG to that reading for each module.
    private final double encoderOffsetDeg;

    /**
     * @param hardwareMap      The OpMode's hardwareMap.
     * @param driveMotorName   Config name for the drive DcMotor.
     * @param steerServoName   Config name for the Axon MAX Mk2 CRServo.
     * @param encoderName      Config name for the Axon analog encoder input.
     * @param reverseMotor     True if the drive motor spins the wrong way.
     * @param encoderOffsetDeg Degrees to subtract so 0° = forward.
     */
    public SwerveModule(HardwareMap hardwareMap,
                        String driveMotorName,
                        String steerServoName,
                        String encoderName,
                        boolean reverseMotor,
                        double encoderOffsetDeg) {

        driveMotor   = hardwareMap.get(DcMotor.class,     driveMotorName);
        steerServo   = hardwareMap.get(CRServo.class,     steerServoName);
        steerEncoder = hardwareMap.get(AnalogInput.class, encoderName);

        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setDirection(
                reverseMotor ? DcMotorSimple.Direction.REVERSE
                             : DcMotorSimple.Direction.FORWARD);

        this.encoderOffsetDeg = encoderOffsetDeg;
        pidLastTimeMs = System.currentTimeMillis();

        // Seed the wrap tracker with the encoder's current raw reading so
        // getCurrentAngle() returns the correct value on the very first call.
        lastRawEncoderDeg    = (steerEncoder.getVoltage() / 3.3) * 360.0;
        continuousEncoderDeg = lastRawEncoderDeg;
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Commands the module to a target heading and drive power.
     * Call this every loop iteration.
     *
     * @param targetAngleDeg Desired wheel-forward direction in degrees,
     *                       where 0° = robot-forward, 90° = robot-left.
     *                       Input is automatically normalized to [0, 360).
     * @param drivePower     Desired drive output, −1.0 to +1.0.
     */
    public void set(double targetAngleDeg, double drivePower) {
        double currentAngle = getCurrentAngle();
        targetAngleDeg = normalizeAngle(targetAngleDeg);

        // ── Module optimization ──────────────────────────────────────────────
        // If the shortest path to the target is >90°, flip 180° and reverse
        // the drive motor instead — the wheel gets there twice as fast.
        double error = shortestAngularError(currentAngle, targetAngleDeg);
        if (Math.abs(error) > 90.0) {
            targetAngleDeg = normalizeAngle(targetAngleDeg + 180.0);
            drivePower     = -drivePower;
            error          = shortestAngularError(currentAngle, targetAngleDeg);
        }

        // ── Steering PID ─────────────────────────────────────────────────────
        long   nowMs = System.currentTimeMillis();
        double dt    = Math.max((nowMs - pidLastTimeMs) / 1000.0, 0.001);

        pidIntegral = clamp(pidIntegral + error * dt, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);
        double derivative = (error - pidLastError) / dt;
        double steerPower  = kP * error + kI * pidIntegral + kD * derivative;
        steerPower = clamp(steerPower, -1.0, 1.0);

        pidLastError  = error;
        pidLastTimeMs = nowMs;

        steerServo.setPower(steerPower);

        // ── Cosine drive scaling ──────────────────────────────────────────────
        // Reduce drive output proportionally while the module is still turning.
        // At 0° error the wheel drives at full power; at 90° it drives at 0.
        double cosineScale = Math.cos(Math.toRadians(error));
        double scaledPower = drivePower * cosineScale;

        driveMotor.setPower(Math.abs(scaledPower) < DRIVE_DEADBAND ? 0.0 : scaledPower);
    }

    /**
     * Returns the current module heading in [0, 360) degrees,
     * corrected by the encoder zero offset.
     */
    public double getCurrentAngle() {
        // ── Step 1: read raw servo-shaft angle (0–360°) ──────────────────
        double rawDeg = (steerEncoder.getVoltage() / 3.3) * 360.0;

        // ── Step 2: detect encoder wrap-arounds ──────────────────────────
        // The encoder resets to 0° every servo revolution.  A jump larger
        // than ±180° in one loop means the 0/360 boundary was crossed.
        double delta = rawDeg - lastRawEncoderDeg;
        if      (delta >  180.0) delta -= 360.0;  // crossed 360→0 backward
        else if (delta < -180.0) delta += 360.0;  // crossed 0→360 forward
        continuousEncoderDeg += delta;
        lastRawEncoderDeg     = rawDeg;

        // ── Step 3: convert servo degrees → pod degrees via belt ratio ───
        // BELT_RATIO = 24/56 ≈ 0.4286
        // One servo revolution (360°) turns the pod only 154.3°.
        double podDeg = continuousEncoderDeg * BELT_RATIO;

        // ── Step 4: apply zero-offset and normalize to [0, 360) ──────────
        return normalizeAngle(podDeg - encoderOffsetDeg);
    }

    /** Returns the raw encoder voltage (useful for finding ENCODER_OFFSET_DEG). */
    public double getRawEncoderVoltage() {
        return steerEncoder.getVoltage();
    }

    /** Cuts power to both the drive motor and the steering servo. */
    public void stop() {
        driveMotor.setPower(0.0);
        steerServo.setPower(0.0);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    /** Wraps an angle into [0, 360). */
    private static double normalizeAngle(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    /**
     * Shortest signed error from {@code current} to {@code target}, in (−180, 180].
     */
    private static double shortestAngularError(double current, double target) {
        double err = target - current;
        while (err >  180.0) err -= 360.0;
        while (err < -180.0) err += 360.0;
        return err;
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
