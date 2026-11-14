package frc.robot.configs;

import edu.wpi.first.math.util.Units;

/**
 * ============================================================================
 *                          SWERVE DRIVE CONFIGURATION
 * ============================================================================
 * Central configuration file for the swerve drivetrain.
 *
 * HARDWARE SETUP:
 *   - Swerve Modules: Swerve Drive Specialties (SDS) MK5n (R1 Drive Ratio)
 *   - Drive Motors:   Kraken X60 (Talon FX)
 *   - Steer Motors:   Kraken X44 (Talon FX)
 *   - Absolute Enc:   CTRE CANcoder
 *   - IMU / Gyro:     CTRE Pigeon 2
 *   - Main Controller:RoboRIO 1
 *
 * Modify settings here to tune and calibrate the drivetrain.
 * ============================================================================
 */
public final class SwerveConfig {

    private SwerveConfig() {}

    /* ========================================================================
     * 1. CAN BUS CONFIGURATION (RoboRIO 1)
     * ======================================================================== */
    /**
     * CAN Bus name.
     *  - For the default RoboRIO CAN bus: use "rio" or "" (empty string).
     *  - If using a CTRE CANivore: set to "canivore" or the name given in Tuner X.
     *
     * NOTE FOR RoboRIO 1:
     *   RoboRIO 1 uses standard CAN 2.0 (1 Mbps). To prevent CAN bus saturation
     *   with 13 CTRE devices (8 TalonFX + 4 CANcoder + 1 Pigeon2), odometry
     *   update frequency is configured to 100 Hz or standard WPILib loop (50 Hz).
     */
    public static final String CAN_BUS_NAME = "rio";

    /** Pigeon 2 IMU CAN ID */
    public static final int PIGEON2_CAN_ID = 13;


    /* ========================================================================
     * 2. MODULE CAN IDs AND CANCODER OFFSETS
     * ========================================================================
     * HOW TO ZERO / CALIBRATE MODULE OFFSETS:
     *  1. Align all wheels mechanically straight (bevel gears facing same direction,
     *     typically facing LEFT or RIGHT per team standard).
     *  2. Open Phoenix Tuner X, find each CANcoder, read the "Absolute Position"
     *     (in rotations: range -0.5 to +0.5 or 0.0 to 1.0).
     *  3. Negate that value (or enter the measured offset) into the respective
     *     offset constant below.
     * ======================================================================== */

    // --- FRONT LEFT MODULE ---
    public static final int FL_DRIVE_MOTOR_ID     = 1;
    public static final int FL_STEER_MOTOR_ID     = 2;
    public static final int FL_CANCODER_ID        = 9;
    /** CANcoder magnet offset in rotations (-0.5 to 0.5) */
    public static final double FL_CANCODER_OFFSET = 0.0;

    // --- FRONT RIGHT MODULE ---
    public static final int FR_DRIVE_MOTOR_ID     = 3;
    public static final int FR_STEER_MOTOR_ID     = 4;
    public static final int FR_CANCODER_ID        = 10;
    /** CANcoder magnet offset in rotations (-0.5 to 0.5) */
    public static final double FR_CANCODER_OFFSET = 0.0;

    // --- BACK LEFT MODULE ---
    public static final int BL_DRIVE_MOTOR_ID     = 5;
    public static final int BL_STEER_MOTOR_ID     = 6;
    public static final int BL_CANCODER_ID        = 11;
    /** CANcoder magnet offset in rotations (-0.5 to 0.5) */
    public static final double BL_CANCODER_OFFSET = 0.0;

    // --- BACK RIGHT MODULE ---
    public static final int BR_DRIVE_MOTOR_ID     = 7;
    public static final int BR_STEER_MOTOR_ID     = 8;
    public static final int BR_CANCODER_ID        = 12;
    /** CANcoder magnet offset in rotations (-0.5 to 0.5) */
    public static final double BR_CANCODER_OFFSET = 0.0;


    /* ========================================================================
     * 3. SDS MK5n MODULE SPECIFICATIONS & GEAR RATIOS
     * ======================================================================== */
    /**
     * MK5n Drive Gear Ratios:
     *  - R1: (54/12) * (25/32) * (30/15) = 4.5 * 0.78125 * 2.0 = 7.03125 : 1
     *  - R2: (54/14) * (25/32) * (30/15) = 3.857 * 0.78125 * 2.0 = 6.02678 : 1
     *  - R3: (54/16) * (25/32) * (30/15) = 3.375 * 0.78125 * 2.0 = 5.27343 : 1
     */
    public static final double DRIVE_GEAR_RATIO = 7.03125; // SDS MK5n R1

    /**
     * MK5n Steering (Azimuth) Gear Ratio:
     * Enclosed gear train ratio is 287:11 (approx 26.0909:1).
     */
    public static final double STEER_GEAR_RATIO = 287.0 / 11.0;

    /**
     * MK5n Coupling Gear Ratio:
     * For modules where steer gear does not couple into drive wheel rotation, ratio is 0.
     */
    public static final double COUPLING_GEAR_RATIO = 0.0;

    /** Nominal wheel radius (4.0 inch diameter wheel -> 2.0 inch radius) */
    public static final double WHEEL_RADIUS_INCHES = 2.0;
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(WHEEL_RADIUS_INCHES);


    /* ========================================================================
     * 4. PHYSICAL ROBOT DIMENSIONS (CHASSIS GEOMETRY)
     * ========================================================================
     * Distance between center of left wheels and center of right wheels (Track Width)
     * Distance between center of front wheels and center of back wheels (Wheel Base)
     * Units: Meters
     * ======================================================================== */
    public static final double TRACK_WIDTH_X_METERS = Units.inchesToMeters(22.75); // Front-to-Back wheelbase
    public static final double TRACK_WIDTH_Y_METERS = Units.inchesToMeters(22.75); // Left-to-Right trackwidth

    // Module X and Y offsets from center of robot (Standard WPILib: +X = forward, +Y = left)
    public static final double FRONT_LEFT_X  =  TRACK_WIDTH_X_METERS / 2.0;
    public static final double FRONT_LEFT_Y  =  TRACK_WIDTH_Y_METERS / 2.0;

    public static final double FRONT_RIGHT_X =  TRACK_WIDTH_X_METERS / 2.0;
    public static final double FRONT_RIGHT_Y = -TRACK_WIDTH_Y_METERS / 2.0;

    public static final double BACK_LEFT_X   = -TRACK_WIDTH_X_METERS / 2.0;
    public static final double BACK_LEFT_Y   =  TRACK_WIDTH_Y_METERS / 2.0;

    public static final double BACK_RIGHT_X  = -TRACK_WIDTH_X_METERS / 2.0;
    public static final double BACK_RIGHT_Y  = -TRACK_WIDTH_Y_METERS / 2.0;


    /* ========================================================================
     * 5. MOTOR AND SENSOR INVERSIONS
     * ======================================================================== */
    /** Invert drive motors across the robot */
    public static final boolean INVERT_LEFT_SIDE_DRIVE  = false;
    public static final boolean INVERT_RIGHT_SIDE_DRIVE = true;

    /** Steer motor inversion for SDS MK5n */
    public static final boolean STEER_MOTOR_INVERTED = true;

    /** CANcoder inversion */
    public static final boolean CANCODER_INVERTED = false;


    /* ========================================================================
     * 6. CURRENT LIMITS & BROWNOUT PROTECTION (RoboRIO 1 CRITICAL)
     * ========================================================================
     * RoboRIO 1 is sensitive to system voltage drops below 6.8V.
     * Kraken X60 can draw over 300A at stall. Strict supply and stator current
     * limits safeguard the battery and avoid brownouts during fast direction changes.
     * ======================================================================== */
    // Kraken X60 (Drive Motor) Current Limits
    public static final boolean DRIVE_STATOR_LIMIT_ENABLE = true;
    public static final double  DRIVE_STATOR_CURRENT_LIMIT = 80.0; // Amps (Torque limiter)

    public static final boolean DRIVE_SUPPLY_LIMIT_ENABLE = true;
    public static final double  DRIVE_SUPPLY_CURRENT_LIMIT = 50.0; // Amps (Battery draw limiter)
    public static final double  DRIVE_SUPPLY_CURRENT_THRESHOLD = 65.0; // Amps (Burst threshold)
    public static final double  DRIVE_SUPPLY_TIME_THRESHOLD = 0.1; // Seconds

    // Kraken X44 (Steer Motor) Current Limits
    public static final boolean STEER_STATOR_LIMIT_ENABLE = true;
    public static final double  STEER_STATOR_CURRENT_LIMIT = 40.0; // Amps

    public static final boolean STEER_SUPPLY_LIMIT_ENABLE = true;
    public static final double  STEER_SUPPLY_CURRENT_LIMIT = 30.0; // Amps
    public static final double  STEER_SUPPLY_CURRENT_THRESHOLD = 40.0; // Amps
    public static final double  STEER_SUPPLY_TIME_THRESHOLD = 0.1; // Seconds


    /* ========================================================================
     * 7. CLOSED-LOOP PID AND FEEDFORWARD GAINS
     * ======================================================================== */
    // Steer Motor PID (Position Closed Loop, Rotations)
    public static final double STEER_KP = 100.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.5;
    public static final double STEER_KS = 0.1; // Static friction feedforward (Volts)
    public static final double STEER_KV = 1.91; // Velocity feedforward (Volts per RPS)
    public static final double STEER_KA = 0.0;

    // Drive Motor PID (Velocity Closed Loop, RPS)
    public static final double DRIVE_KP = 0.12;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.20; // Static friction feedforward (Volts)
    public static final double DRIVE_KV = 0.12; // Volts per RPS
    public static final double DRIVE_KA = 0.0;


    /* ========================================================================
     * 8. SPEED, ACCELERATION, AND SLEW LIMITS
     * ======================================================================== */
    /**
     * Theoretical free speed at 12V with Kraken X60 (6000 RPM) and MK5n R1:
     * 6000 RPM / 60 / 7.03125 * (2 * PI * 0.0508 m) ≈ 4.54 m/s (14.9 ft/s)
     */
    public static final double MAX_LINEAR_SPEED_MPS = 4.5;

    /** Maximum angular rate (radians per second): ~1.5 to 2 full spins/sec */
    public static final double MAX_ANGULAR_RATE_RAD_PER_SEC = 2.0 * Math.PI * 1.5;

    /** Multipliers for driver speed profiles */
    public static final double SLOW_MODE_MULTIPLIER  = 0.35; // 35% speed for fine alignment
    public static final double TURBO_MODE_MULTIPLIER = 1.00; // Full speed

    /** Slew rate limiters (Max acceleration to prevent wheel slipping/tipping) */
    public static final double TRANSLATIONAL_SLEW_RATE = 14.0; // m/s^2
    public static final double ROTATIONAL_SLEW_RATE    = 18.0; // rad/s^2


    /* ========================================================================
     * 9. DRIVER CONTROLLER CONFIGURATION
     * ======================================================================== */
    public static final double STICK_DEADBAND = 0.08; // 8% stick deadband
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;


    /* ========================================================================
     * 10. ODOMETRY & CAN UPDATE FREQUENCIES (RoboRIO 1 OPTIMIZATION)
     * ======================================================================== */
    /**
     * Odometry update frequency:
     *  - With CANivore: standard is 250 Hz.
     *  - With RoboRIO 1 onboard CAN: 100 Hz or standard WPILib loop (50 Hz)
     *    is recommended to preserve CAN bus bandwidth.
     */
    public static final double ODOMETRY_UPDATE_FREQUENCY_HZ = 100.0;
}
