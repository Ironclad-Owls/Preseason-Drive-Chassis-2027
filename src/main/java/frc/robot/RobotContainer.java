// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
 

    public static final double MIN_SPEED_MPS = 0.3;
    public static final double MAX_SPEED_MPS = 3.0;
    public static final double MAX_ACCEL_MPS2 = 0.7 * 1;
    public static final double MAX_ROT_RAD_PER_SEC = 2.5 * Math.PI;
    public static final double MAX_ROT_ACCEL_RAD_PER_SEC2 = 12.0;
    public static final double DEADBAND = 0.08;
    public static final double ROTATION_DEADBAND = 0.15;
    public static final double ANGLE_DEADBAND_RAD = Math.toRadians(2.5);
    public static final double SLOW_MULTIPLIER = 0.35;
    public static final double ANGLE_SMOOTHING = 0.15;
    public static final double ANGLE_SNAP_RAD = Math.toRadians(45.0);

    private final SlewRateLimiter speedRate =
        new SlewRateLimiter(MAX_ACCEL_MPS2, -100.0, 0.0);
    private final SlewRateLimiter omegaRate =
        new SlewRateLimiter(MAX_ROT_ACCEL_RAD_PER_SEC2);
    private boolean wasMoving = false;
    private double smoothedAngle = 0.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withDesaturateWheelSpeeds(true);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double slow = joystick.leftBumper().getAsBoolean()
                    ? SLOW_MULTIPLIER : 1.0;
                double rawFwd = -joystick.getLeftY();
                double rawStrafe = -joystick.getLeftX();
                double rawTurn = MathUtil.applyDeadband(-joystick.getRightX(),
                    ROTATION_DEADBAND) * MAX_ROT_RAD_PER_SEC * slow;
                SmartDashboard.putNumber("Driver/RawFwd", rawFwd);
                SmartDashboard.putNumber("Driver/RawStrafe", rawStrafe);
                SmartDashboard.putNumber("Driver/TurnCmd", rawTurn);

                double rawMag = Math.hypot(rawFwd, rawStrafe);
                double mag = MathUtil.applyDeadband(rawMag, DEADBAND);
                double vx = 0.0;
                double vy = 0.0;

                if (mag > 0.0) {
                    double maxAllowed = MAX_SPEED_MPS * slow;
                    double minAllowed = Math.min(MIN_SPEED_MPS, maxAllowed);
                    double targetAngle = Math.atan2(rawStrafe, rawFwd);

                    if (!wasMoving) {
                        speedRate.reset(minAllowed);
                        smoothedAngle = targetAngle;
                        wasMoving = true;
                    } else {
                        double angleError = MathUtil.angleModulus(targetAngle - smoothedAngle);
                        if (Math.abs(angleError) > ANGLE_SNAP_RAD) {
                            smoothedAngle = targetAngle;
                        } else if (Math.abs(angleError) > ANGLE_DEADBAND_RAD) {
                            smoothedAngle = MathUtil.angleModulus(
                                smoothedAngle + ANGLE_SMOOTHING * angleError
                            );
                        }
                    }

                    double targetSpeed = MathUtil.interpolate(
                        minAllowed, maxAllowed, Math.min(mag, 1.0)
                    );
                    double currentSpeed = speedRate.calculate(targetSpeed);

                    vx = currentSpeed * Math.cos(smoothedAngle);
                    vy = currentSpeed * Math.sin(smoothedAngle);
                } else {
                    wasMoving = false;
                    speedRate.reset(0.0);
                    vx = 0.0;
                    vy = 0.0;
                }

                return drive.withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(omegaRate.calculate(rawTurn));
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
