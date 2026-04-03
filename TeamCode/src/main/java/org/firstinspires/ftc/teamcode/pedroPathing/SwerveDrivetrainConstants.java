package org.firstinspires.ftc.teamcode.pedroPathing;

/**
 * SwerveDrivetrainConstants
 *
 * Constants class for the coaxial swerve drivetrain.
 * Adjust these values to match your physical robot.
 *
 * Hardware:
 *   - Drive motors:  GoBilda (yellow jacket or similar)
 *   - Pivot servos:  Axon MAX / MINI (programmable, up to 355°)
 *
 * PedroPathing Custom Drivetrain docs:
 *   https://pedropathing.com/docs/pathing/custom/drivetrain
 */
public class SwerveDrivetrainConstants {

    // -------------------------------------------------------------------------
    // Hardware map names — must match your robot configuration exactly
    // -------------------------------------------------------------------------

    /** Drive motor hardware map names, ordered: FL, FR, BL, BR */
    public static final String[] DRIVE_MOTOR_NAMES = {
            "frontLeftDrive",
            "frontRightDrive",
            "backLeftDrive",
            "backRightDrive"
    };

    /**
     * Pivot servo hardware map names, ordered: FL, FR, BL, BR.
     *
     * These are Axon servos configured in standard servo mode.
     * Configure them with the Axon Programmer to use full 355° range,
     * then select "Full Range Servo" in the REV Hub config so the
     * FTC SDK maps 0.0–1.0 across the full 500–2500 µs PWM range.
     *
     * Axon Servo Programmer docs:
     *   https://docs.axon-robotics.com/axon-servos/servo-programmer
     */
    public static final String[] PIVOT_SERVO_NAMES = {
            "frontLeftPivot",
            "frontRightPivot",
            "backLeftPivot",
            "backRightPivot"
    };

    // -------------------------------------------------------------------------
    // Drive motor direction
    // Flip a motor if its wheel spins the wrong way for a given pod angle.
    // -------------------------------------------------------------------------

    /** true = REVERSE, false = FORWARD; ordered FL, FR, BL, BR */
    public static final boolean[] DRIVE_MOTOR_REVERSED = {
            false,  // FL
            true,   // FR  — typically mirrored
            false,  // BL
            true    // BR  — typically mirrored
    };

    // -------------------------------------------------------------------------
    // Robot geometry
    // Measure center-to-center between swerve pods (in inches or consistent unit).
    // Used to compute each pod's contribution to the robot heading correction.
    // -------------------------------------------------------------------------

    /** Distance between left and right pod centers (track width), same unit as Pedro field coords */
    public static final double TRACK_WIDTH  = 12.0; // inches — TUNE THIS

    /** Distance between front and back pod centers (wheelbase) */
    public static final double WHEEL_BASE   = 12.0; // inches — TUNE THIS

    // -------------------------------------------------------------------------
    // Axon servo mapping
    //
    // The Axon servo's setPosition() range is 0.0 → 1.0 (standard FTC Servo API).
    // With the Axon Programmer set to 355° and "Full Range Servo" selected in
    // the hub config, 0.0 ≈ 0° and 1.0 ≈ 355°.
    //
    // We map 0° → 355° of physical travel to the 0.0 → 1.0 API range.
    // The formula to convert a target angle θ (radians, 0 to 2π) to servo position:
    //
    //   position = (θ / SERVO_MAX_ANGLE_RAD) * (SERVO_MAX_POSITION - SERVO_MIN_POSITION)
    //              + SERVO_MIN_POSITION
    //
    // If your Axon is programmed for a different range (e.g. 180°), change
    // SERVO_MAX_ANGLE_DEG accordingly.
    // -------------------------------------------------------------------------

    /** Physical travel of each Axon servo in degrees (match what you programmed with Axon Programmer) */
    public static final double SERVO_MAX_ANGLE_DEG = 355.0;

    /** Servo API position corresponding to 0° */
    public static final double SERVO_MIN_POSITION  = 0.0;

    /** Servo API position corresponding to SERVO_MAX_ANGLE_DEG */
    public static final double SERVO_MAX_POSITION  = 1.0;

    /**
     * Physical angle offset (degrees) added to each pod so that servo position 0.5
     * (the mechanical center) aligns with the robot's forward direction (+X).
     * Ordered FL, FR, BL, BR. Tune these after assembly.
     */
    public static final double[] SERVO_ZERO_OFFSET_DEG = {
            0.0,  // FL — set after mounting
            0.0,  // FR
            0.0,  // BL
            0.0   // BR
    };

    // -------------------------------------------------------------------------
    // Speed limits
    // -------------------------------------------------------------------------

    /** Maximum allowed drive motor power (0.0 – 1.0). Reduce if you need slower movement. */
    public static final double MAX_DRIVE_POWER = 1.0;

    /**
     * Minimum drive motor power threshold.
     * Powers below this magnitude are zeroed to prevent motor hum / heat buildup.
     */
    public static final double MIN_DRIVE_POWER_THRESHOLD = 0.01;

    /**
     * When the pod needs to rotate more than this many degrees to reach its target,
     * the drive motor is reversed and the pod rotates to the opposite angle instead,
     * keeping pivot travel ≤ 90°. This is the standard swerve "optimization."
     */
    public static final double OPTIMIZATION_THRESHOLD_DEG = 90.0;
}
