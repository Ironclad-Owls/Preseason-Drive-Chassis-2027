package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.configs.SwerveConfig;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command factory for driving modes.
 */
public final class DriveCommands {

    private DriveCommands() {}

    /**
     * Standard Field-Centric drive command.
     *
     * @param drivetrain Drivetrain subsystem
     * @param translationX Forward / Backward supplier (-1.0 to 1.0, forward positive)
     * @param translationY Left / Right supplier (-1.0 to 1.0, left positive)
     * @param rotation Rotational supplier (-1.0 to 1.0, CCW positive)
     * @param slowMode Active when slow mode is toggled/held
     * @return Continuous driving Command
     */
    public static Command fieldCentricDrive(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotation,
            BooleanSupplier slowMode) {

        final SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric()
            .withDeadband(SwerveConfig.MAX_LINEAR_SPEED_MPS * SwerveConfig.STICK_DEADBAND)
            .withRotationalDeadband(SwerveConfig.MAX_ANGULAR_RATE_RAD_PER_SEC * SwerveConfig.STICK_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        final SlewRateLimiter xLimiter = new SlewRateLimiter(SwerveConfig.TRANSLATIONAL_SLEW_RATE);
        final SlewRateLimiter yLimiter = new SlewRateLimiter(SwerveConfig.TRANSLATIONAL_SLEW_RATE);
        final SlewRateLimiter rotLimiter = new SlewRateLimiter(SwerveConfig.ROTATIONAL_SLEW_RATE);

        return drivetrain.applyRequest(() -> {
            // Apply deadbands
            double x = MathUtil.applyDeadband(translationX.getAsDouble(), SwerveConfig.STICK_DEADBAND);
            double y = MathUtil.applyDeadband(translationY.getAsDouble(), SwerveConfig.STICK_DEADBAND);
            double rot = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConfig.STICK_DEADBAND);

            // Apply quadratic curvature for precision around center
            x = Math.copySign(x * x, x);
            y = Math.copySign(y * y, y);
            rot = Math.copySign(rot * rot, rot);

            // Speed multiplier
            double multiplier = slowMode.getAsBoolean()
                ? SwerveConfig.SLOW_MODE_MULTIPLIER
                : SwerveConfig.TURBO_MODE_MULTIPLIER;

            // Scale to physical velocities
            double targetVx = xLimiter.calculate(x * SwerveConfig.MAX_LINEAR_SPEED_MPS * multiplier);
            double targetVy = yLimiter.calculate(y * SwerveConfig.MAX_LINEAR_SPEED_MPS * multiplier);
            double targetRot = rotLimiter.calculate(rot * SwerveConfig.MAX_ANGULAR_RATE_RAD_PER_SEC * multiplier);

            return request
                .withVelocityX(targetVx)
                .withVelocityY(targetVy)
                .withRotationalRate(targetRot);
        });
    }

    /**
     * Robot-Centric drive command. Useful for intake alignment and pit testing.
     */
    public static Command robotCentricDrive(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            DoubleSupplier rotation,
            BooleanSupplier slowMode) {

        final SwerveRequest.RobotCentric request = new SwerveRequest.RobotCentric()
            .withDeadband(SwerveConfig.MAX_LINEAR_SPEED_MPS * SwerveConfig.STICK_DEADBAND)
            .withRotationalDeadband(SwerveConfig.MAX_ANGULAR_RATE_RAD_PER_SEC * SwerveConfig.STICK_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        return drivetrain.applyRequest(() -> {
            double x = MathUtil.applyDeadband(translationX.getAsDouble(), SwerveConfig.STICK_DEADBAND);
            double y = MathUtil.applyDeadband(translationY.getAsDouble(), SwerveConfig.STICK_DEADBAND);
            double rot = MathUtil.applyDeadband(rotation.getAsDouble(), SwerveConfig.STICK_DEADBAND);

            double multiplier = slowMode.getAsBoolean()
                ? SwerveConfig.SLOW_MODE_MULTIPLIER
                : SwerveConfig.TURBO_MODE_MULTIPLIER;

            return request
                .withVelocityX(x * SwerveConfig.MAX_LINEAR_SPEED_MPS * multiplier)
                .withVelocityY(y * SwerveConfig.MAX_LINEAR_SPEED_MPS * multiplier)
                .withRotationalRate(rot * SwerveConfig.MAX_ANGULAR_RATE_RAD_PER_SEC * multiplier);
        });
    }

    /**
     * Field-Centric drive with automatic heading snapping/locking.
     * Keeps the robot facing targetAngle while translating with joysticks.
     */
    public static Command faceAngleDrive(
            CommandSwerveDrivetrain drivetrain,
            DoubleSupplier translationX,
            DoubleSupplier translationY,
            Rotation2d targetAngle,
            BooleanSupplier slowMode) {

        final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(SwerveConfig.MAX_LINEAR_SPEED_MPS * SwerveConfig.STICK_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Heading PID gains
        request.HeadingController.setPID(7.0, 0.0, 0.3);
        request.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        return drivetrain.applyRequest(() -> {
            double x = MathUtil.applyDeadband(translationX.getAsDouble(), SwerveConfig.STICK_DEADBAND);
            double y = MathUtil.applyDeadband(translationY.getAsDouble(), SwerveConfig.STICK_DEADBAND);

            double multiplier = slowMode.getAsBoolean()
                ? SwerveConfig.SLOW_MODE_MULTIPLIER
                : SwerveConfig.TURBO_MODE_MULTIPLIER;

            return request
                .withVelocityX(x * SwerveConfig.MAX_LINEAR_SPEED_MPS * multiplier)
                .withVelocityY(y * SwerveConfig.MAX_LINEAR_SPEED_MPS * multiplier)
                .withTargetDirection(targetAngle);
        });
    }

    /**
     * Defense brake: locks wheels into an 'X' pattern so the robot cannot easily be pushed.
     */
    public static Command xBrake(CommandSwerveDrivetrain drivetrain) {
        return drivetrain.xBrakeCommand();
    }

    /**
     * Zeros the gyro orientation to field-forward.
     */
    public static Command zeroGyro(CommandSwerveDrivetrain drivetrain) {
        return Commands.runOnce(drivetrain::zeroHeading, drivetrain);
    }
}
