package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import frc.robot.configs.SwerveConfig;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * TunerConstants builds CTRE Phoenix 6 Swerve configuration objects
 * directly using values defined in {@link SwerveConfig}.
 *
 * Hardware:
 *   - SDS MK5n R1 (Drive: 7.03125:1, Steer: 287:11)
 *   - Drive: Kraken X60 (Talon FX)
 *   - Steer: Kraken X44 (Talon FX)
 *   - CANcoder
 *   - Pigeon 2
 *   - RoboRIO 1
 */
public class TunerConstants {

    // CAN bus specification (e.g., "rio" for RoboRIO 1)
    public static final CANBus kCANBus = new CANBus(SwerveConfig.CAN_BUS_NAME);

    // Closed-loop PID & Feedforward gains for Steer (Slot 0)
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(SwerveConfig.STEER_KP)
        .withKI(SwerveConfig.STEER_KI)
        .withKD(SwerveConfig.STEER_KD)
        .withKS(SwerveConfig.STEER_KS)
        .withKV(SwerveConfig.STEER_KV)
        .withKA(SwerveConfig.STEER_KA);

    // Closed-loop PID & Feedforward gains for Drive (Slot 0)
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(SwerveConfig.DRIVE_KP)
        .withKI(SwerveConfig.DRIVE_KI)
        .withKD(SwerveConfig.DRIVE_KD)
        .withKS(SwerveConfig.DRIVE_KS)
        .withKV(SwerveConfig.DRIVE_KV)
        .withKA(SwerveConfig.DRIVE_KA);

    // Steer Motor Configurations (Kraken X44)
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(SwerveConfig.STEER_STATOR_LIMIT_ENABLE)
            .withStatorCurrentLimit(SwerveConfig.STEER_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(SwerveConfig.STEER_SUPPLY_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SwerveConfig.STEER_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SwerveConfig.STEER_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerTime(SwerveConfig.STEER_SUPPLY_TIME_THRESHOLD));

    // Drive Motor Configurations (Kraken X60)
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimitEnable(SwerveConfig.DRIVE_STATOR_LIMIT_ENABLE)
            .withStatorCurrentLimit(SwerveConfig.DRIVE_STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimitEnable(SwerveConfig.DRIVE_SUPPLY_LIMIT_ENABLE)
            .withSupplyCurrentLimit(SwerveConfig.DRIVE_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerLimit(SwerveConfig.DRIVE_SUPPLY_CURRENT_LIMIT)
            .withSupplyCurrentLowerTime(SwerveConfig.DRIVE_SUPPLY_TIME_THRESHOLD));

    // CANcoder initial configuration
    private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();

    // Pigeon 2 configuration
    private static final Pigeon2Configuration pigeonConfigs = new Pigeon2Configuration();

    // Swerve Drivetrain Constants
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(kCANBus.getName())
        .withPigeon2Id(SwerveConfig.PIGEON2_CAN_ID)
        .withPigeon2Configs(pigeonConfigs);

    // Swerve Module Constants Factory configured for SDS MK5n R1
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(SwerveConfig.DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(SwerveConfig.STEER_GEAR_RATIO)
            .withCouplingGearRatio(SwerveConfig.COUPLING_GEAR_RATIO)
            .withWheelRadius(SwerveConfig.WHEEL_RADIUS_METERS)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSlipCurrent(SwerveConfig.DRIVE_STATOR_CURRENT_LIMIT)
            .withSpeedAt12Volts(SwerveConfig.MAX_LINEAR_SPEED_MPS)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withCANcoderInitialConfigs(cancoderInitialConfigs);

    // Front Left Module
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            SwerveConfig.FL_STEER_MOTOR_ID,
            SwerveConfig.FL_DRIVE_MOTOR_ID,
            SwerveConfig.FL_CANCODER_ID,
            SwerveConfig.FL_CANCODER_OFFSET,
            SwerveConfig.FRONT_LEFT_X,
            SwerveConfig.FRONT_LEFT_Y,
            SwerveConfig.INVERT_LEFT_SIDE_DRIVE,
            SwerveConfig.STEER_MOTOR_INVERTED,
            SwerveConfig.CANCODER_INVERTED
        );

    // Front Right Module
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            SwerveConfig.FR_STEER_MOTOR_ID,
            SwerveConfig.FR_DRIVE_MOTOR_ID,
            SwerveConfig.FR_CANCODER_ID,
            SwerveConfig.FR_CANCODER_OFFSET,
            SwerveConfig.FRONT_RIGHT_X,
            SwerveConfig.FRONT_RIGHT_Y,
            SwerveConfig.INVERT_RIGHT_SIDE_DRIVE,
            SwerveConfig.STEER_MOTOR_INVERTED,
            SwerveConfig.CANCODER_INVERTED
        );

    // Back Left Module
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            SwerveConfig.BL_STEER_MOTOR_ID,
            SwerveConfig.BL_DRIVE_MOTOR_ID,
            SwerveConfig.BL_CANCODER_ID,
            SwerveConfig.BL_CANCODER_OFFSET,
            SwerveConfig.BACK_LEFT_X,
            SwerveConfig.BACK_LEFT_Y,
            SwerveConfig.INVERT_LEFT_SIDE_DRIVE,
            SwerveConfig.STEER_MOTOR_INVERTED,
            SwerveConfig.CANCODER_INVERTED
        );

    // Back Right Module
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            SwerveConfig.BR_STEER_MOTOR_ID,
            SwerveConfig.BR_DRIVE_MOTOR_ID,
            SwerveConfig.BR_CANCODER_ID,
            SwerveConfig.BR_CANCODER_OFFSET,
            SwerveConfig.BACK_RIGHT_X,
            SwerveConfig.BACK_RIGHT_Y,
            SwerveConfig.INVERT_RIGHT_SIDE_DRIVE,
            SwerveConfig.STEER_MOTOR_INVERTED,
            SwerveConfig.CANCODER_INVERTED
        );

    /**
     * Factory helper to create the configured CommandSwerveDrivetrain instance.
     * Uses optimized odometry frequency for RoboRIO 1.
     */
    public static CommandSwerveDrivetrain createDrivetrain() {
        return new CommandSwerveDrivetrain(
            DrivetrainConstants,
            SwerveConfig.ODOMETRY_UPDATE_FREQUENCY_HZ,
            FrontLeft,
            FrontRight,
            BackLeft,
            BackRight
        );
    }
}
