package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.util.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.SwerveDrivetrainConstants;

/**
 * SwerveDrivetrain
 *
 * A coaxial (same-axle) swerve drivetrain class compatible with PedroPathing 2.x.
 * Extend {@link Drivetrain} and implement {@code calculateDrive} and {@code runDrive}.
 *
 * Hardware assumptions
 * ─────────────────────
 *  • Four GoBilda drive motors (one per pod, configured as DcMotorEx)
 *  • Four Axon servos as pivot actuators (configured as Servo, Full Range)
 *    - Program each Axon with the Axon Programmer to 355° range
 *    - Select "Full Range Servo" in the REV hub config for full 0.0–1.0 travel
 *
 * PedroPathing custom drivetrain docs:
 *   https://pedropathing.com/docs/pathing/custom/drivetrain
 *
 * Axon servo programmer docs:
 *   https://docs.axon-robotics.com/axon-servos/servo-programmer
 *
 * Swerve kinematics reference (inverse kinematics):
 *   https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
 *
 * Pod index convention used throughout:
 *   0 = Front Left  (FL)
 *   1 = Front Right (FR)
 *   2 = Back Left   (BL)
 *   3 = Back Right  (BR)
 */
public class SwerveDrivetrain extends Drivetrain {

    // ── Hardware ─────────────────────────────────────────────────────────────

    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final Servo[]     pivotServos = new Servo[4];

    // ── Cached state (for optimize-flip logic) ────────────────────────────────

    /** Last commanded angle per pod, in radians [0, 2π) */
    private final double[] lastAngleRad = new double[4];

    // ── Precomputed geometry ──────────────────────────────────────────────────

    /**
     * Half-diagonal of the robot wheelbase rectangle.
     * Used to normalise drive speeds so that |v_pod| ≤ 1.0.
     *
     * R = √( (trackWidth/2)² + (wheelBase/2)² )
     */
    private final double R;

    // ──────────────────────────────────────────────────────────────────────────
    // Constructor
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Initialises all hardware from the provided {@link HardwareMap}.
     *
     * @param hardwareMap the FTC hardware map (typically passed from your OpMode)
     */
    public SwerveDrivetrain(HardwareMap hardwareMap) {
        // Precompute normalisation radius
        double half_w = SwerveDrivetrainConstants.TRACK_WIDTH / 2.0;
        double half_l = SwerveDrivetrainConstants.WHEEL_BASE  / 2.0;
        R = Math.sqrt(half_w * half_w + half_l * half_l);

        // ── Drive motors ──────────────────────────────────────────────────────
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(
                    DcMotorEx.class,
                    SwerveDrivetrainConstants.DRIVE_MOTOR_NAMES[i]);

            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            driveMotors[i].setDirection(
                    SwerveDrivetrainConstants.DRIVE_MOTOR_REVERSED[i]
                            ? DcMotorSimple.Direction.REVERSE
                            : DcMotorSimple.Direction.FORWARD);
        }

        // ── Pivot servos (Axon) ───────────────────────────────────────────────
        for (int i = 0; i < 4; i++) {
            pivotServos[i] = hardwareMap.get(
                    Servo.class,
                    SwerveDrivetrainConstants.PIVOT_SERVO_NAMES[i]);
        }

        // Start with all pods facing forward (servo midpoint ≈ 0.5)
        for (int i = 0; i < 4; i++) {
            lastAngleRad[i] = 0.0;
            setPivotAngle(i, 0.0);
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // PedroPathing abstract method implementations
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * {@inheritDoc}
     *
     * Computes per-pod drive powers and pivot angles using swerve inverse kinematics.
     *
     * PedroPathing passes three robot-centric vectors:
     * <ul>
     *   <li>{@code correctiveVector} – translational + centripetal correction (field-relative,
     *       rotated to robot frame by the follower before this call)</li>
     *   <li>{@code headingVector}    – pure rotation command vector; its magnitude encodes
     *       the desired angular correction power</li>
     *   <li>{@code pathingPower}     – scalar forward/speed override from the path follower</li>
     * </ul>
     *
     * Swerve inverse kinematics (per pod i):
     * <pre>
     *   Vx  = corrective.x + heading_contribution.x_i
     *   Vy  = corrective.y + heading_contribution.y_i
     *
     *   speed_i = √(Vx² + Vy²)
     *   angle_i = atan2(Vy, Vx)
     * </pre>
     *
     * Pod geometry offsets (half track-width / half wheelbase):
     * <pre>
     *   FL: (+L/2, +W/2)   FR: (+L/2, -W/2)
     *   BL: (-L/2, +W/2)   BR: (-L/2, -W/2)
     * </pre>
     *
     * @param correctiveVector robot-centric translational correction vector (x forward, y left)
     * @param headingVector    rotation correction; its magnitude is the rotational power
     * @param pathingPower     scalar drive speed (not used to override vectors here; Pedro
     *                         already incorporates it into the vector magnitudes)
     * @param currentHeading   current robot heading in radians (field frame)
     * @return double[8]: [speed0, angle0, speed1, angle1, speed2, angle2, speed3, angle3]
     */
    @Override
    public double[] calculateDrive(Vector correctiveVector,
                                   Vector headingVector,
                                   double pathingPower,
                                   double currentHeading) {

        // Translation components (robot-centric, Pedro already rotated these)
        double tx = correctiveVector.getXComponent();
        double ty = correctiveVector.getYComponent();

        // Rotation power (magnitude of headingVector)
        double rot = headingVector.getMagnitude()
                * Math.signum(headingVector.getTheta()); // signed rotation

        // Half-dimensions for geometry
        double hW = SwerveDrivetrainConstants.TRACK_WIDTH / 2.0;
        double hL = SwerveDrivetrainConstants.WHEEL_BASE  / 2.0;

        /*
         * Pod corner vectors (offset from robot centre, robot frame):
         *   [Ax, Ay] for each pod when rotated by ω:
         *       tangential contribution = ω × r_perp
         *
         * For a rotation ω about the robot centre:
         *   Δvx_i = -ω * ry_i
         *   Δvy_i = +ω * rx_i
         *
         * Pod positions (rx, ry) in robot frame (x = forward, y = left):
         *   FL (0): ( hL,  hW)   FR (1): ( hL, -hW)
         *   BL (2): (-hL,  hW)   BR (3): (-hL, -hW)
         */
        double[][] podOffset = {
                { hL,  hW},  // FL
                { hL, -hW},  // FR
                {-hL,  hW},  // BL
                {-hL, -hW}   // BR
        };

        double[] speeds = new double[4];
        double[] angles = new double[4];
        double maxSpeed = 1.0; // track for normalisation

        for (int i = 0; i < 4; i++) {
            double rx = podOffset[i][0];
            double ry = podOffset[i][1];

            // Tangential velocity contribution from rotation
            double dvx = -rot * ry / R;
            double dvy =  rot * rx / R;

            double vx = tx + dvx;
            double vy = ty + dvy;

            speeds[i] = Math.sqrt(vx * vx + vy * vy);
            angles[i] = Math.atan2(vy, vx); // radians, [-π, π]

            if (speeds[i] > maxSpeed) maxSpeed = speeds[i];
        }

        // Normalise so no pod exceeds MAX_DRIVE_POWER
        double scale = SwerveDrivetrainConstants.MAX_DRIVE_POWER / maxSpeed;

        double[] output = new double[8]; // [speed0, angle0, speed1, angle1, ...]
        for (int i = 0; i < 4; i++) {
            output[i * 2]     = speeds[i] * scale;
            output[i * 2 + 1] = angles[i];
        }

        return output;
    }

    /**
     * {@inheritDoc}
     *
     * Applies the computed drive powers and pivot angles to hardware.
     *
     * Expects the {@code drivePowers} array from {@link #calculateDrive}:
     * indices [0,2,4,6] = wheel speeds, indices [1,3,5,7] = pivot angles (radians).
     *
     * Applies the "swerve optimization": if the pod must rotate more than 90°,
     * the drive direction is reversed and the pod rotates to the mirrored angle,
     * keeping mechanical travel minimal.
     *
     * @param drivePowers output of calculateDrive — double[8]
     */
    @Override
    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < 4; i++) {
            double speed     = drivePowers[i * 2];
            double targetRad = drivePowers[i * 2 + 1];

            // Normalize angle to [0, 2π)
            targetRad = normalizeAngle(targetRad);

            // ── Swerve optimization ───────────────────────────────────────────
            double delta = angleDifference(targetRad, lastAngleRad[i]);
            double threshRad = Math.toRadians(
                    SwerveDrivetrainConstants.OPTIMIZATION_THRESHOLD_DEG);

            if (Math.abs(delta) > threshRad) {
                // Flip the drive direction and use the opposite angle
                speed     = -speed;
                targetRad = normalizeAngle(targetRad + Math.PI);
            }

            lastAngleRad[i] = targetRad;

            // ── Apply to hardware ─────────────────────────────────────────────
            // Clamp drive power
            speed = Math.abs(speed) < SwerveDrivetrainConstants.MIN_DRIVE_POWER_THRESHOLD
                    ? 0.0
                    : Math.max(-SwerveDrivetrainConstants.MAX_DRIVE_POWER,
                        Math.min(SwerveDrivetrainConstants.MAX_DRIVE_POWER, speed));

            driveMotors[i].setPower(speed);
            setPivotAngle(i, targetRad);
        }
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Helper methods
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Commands pod {@code index} to the given angle.
     *
     * Converts a target angle in radians [0, 2π) to a servo position [0.0, 1.0],
     * accounting for the configured physical range and per-pod zero offset.
     *
     * The mapping formula:
     * <pre>
     *   θ_adjusted = θ_rad + offset_deg × (π/180)
     *   position   = (θ_adjusted / θ_max_rad) × (posMax − posMin) + posMin
     * </pre>
     *
     * @param index     pod index (0=FL, 1=FR, 2=BL, 3=BR)
     * @param angleRad  desired pivot angle in radians [0, 2π)
     */
    private void setPivotAngle(int index, double angleRad) {
        double maxAngleRad = Math.toRadians(SwerveDrivetrainConstants.SERVO_MAX_ANGLE_DEG);
        double offsetRad   = Math.toRadians(
                SwerveDrivetrainConstants.SERVO_ZERO_OFFSET_DEG[index]);

        double adjusted = normalizeAngle(angleRad + offsetRad);

        // Clamp to servo's physical range
        adjusted = Math.min(adjusted, maxAngleRad);

        double range    = SwerveDrivetrainConstants.SERVO_MAX_POSITION
                        - SwerveDrivetrainConstants.SERVO_MIN_POSITION;
        double position = (adjusted / maxAngleRad) * range
                        + SwerveDrivetrainConstants.SERVO_MIN_POSITION;

        // Safety clamp
        position = Math.max(SwerveDrivetrainConstants.SERVO_MIN_POSITION,
                   Math.min(SwerveDrivetrainConstants.SERVO_MAX_POSITION, position));

        pivotServos[index].setPosition(position);
    }

    /**
     * Returns the equivalent angle in [0, 2π).
     */
    private static double normalizeAngle(double rad) {
        rad = rad % (2 * Math.PI);
        if (rad < 0) rad += 2 * Math.PI;
        return rad;
    }

    /**
     * Returns the signed shortest angular difference from {@code from} to {@code to},
     * in the range (-π, π].
     */
    private static double angleDifference(double to, double from) {
        double diff = normalizeAngle(to - from);
        if (diff > Math.PI) diff -= 2 * Math.PI;
        return diff;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Public accessors (useful for telemetry / debugging)
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Returns the raw drive motor array for external access (e.g. localizer encoders).
     * Ordered FL, FR, BL, BR.
     */
    public DcMotorEx[] getDriveMotors() {
        return driveMotors;
    }

    /**
     * Returns the pivot servo array for external access or telemetry.
     * Ordered FL, FR, BL, BR.
     */
    public Servo[] getPivotServos() {
        return pivotServos;
    }
}
