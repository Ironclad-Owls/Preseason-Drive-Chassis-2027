package frc.robot;

import frc.robot.configs.SwerveConfig;

/**
 * General robot constants.
 * Note: Swerve-specific drivetrain parameters are located in {@link SwerveConfig}.
 */
public final class Constants {

    public static final class OperatorConstants {
        // Driver Controller USB Port
        public static final int DRIVER_CONTROLLER_PORT = SwerveConfig.DRIVER_CONTROLLER_PORT;

        // Operator Controller USB Port
        public static final int OPERATOR_CONTROLLER_PORT = SwerveConfig.OPERATOR_CONTROLLER_PORT;
    }

    public static final class AutoConstants {
        public static final double DEFAULT_AUTO_MAX_SPEED_MPS = SwerveConfig.MAX_LINEAR_SPEED_MPS;
        public static final double DEFAULT_AUTO_MAX_ACCEL_MPSS = 3.0;
    }
}
