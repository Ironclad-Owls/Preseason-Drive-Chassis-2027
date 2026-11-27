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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    public static final double MAX_DECEL_MPS2 = 6.0;
    public static final double MAX_ROT_RAD_PER_SEC = 2.5 * Math.PI;
    public static final double MAX_ROT_ACCEL_RAD_PER_SEC2 = 12.0;
    public static final double DEADBAND = 0.08;
    public static final double ROTATION_DEADBAND = 0.15;
    public static final double ANGLE_DEADBAND_RAD = Math.toRadians(2.5);
    public static final double SLOW_MULTIPLIER = 0.35;
    public static final double ANGLE_SMOOTHING = 0.15;
    public static final double ANGLE_SNAP_RAD = Math.toRadians(45.0);

    private final SlewRateLimiter speedRate =
        new SlewRateLimiter(MAX_ACCEL_MPS2, -MAX_DECEL_MPS2, 0.0);
    private boolean wasMoving = false;
    private double smoothedAngle = 0.0;
    private boolean hasTargetHeading = false;
    private Rotation2d targetHeading = Rotation2d.kZero;
    private double targetHeadingRateScale = 1.0;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withDesaturateWheelSpeeds(true);
    private final SwerveRequest.FieldCentricFacingAngle facingDrive = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo)
        .withDesaturateWheelSpeeds(true);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    {
        facingDrive.HeadingController.setPID(5.0, 0.0, 0.0);
        facingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configureBindings();
        configureAutoChooser();
        drivetrain.resetRotation(Rotation2d.k180deg);
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
                double rawTurnX = -joystick.getRightY();
                double rawTurnY = -joystick.getRightX();
                double turnMag = Math.hypot(rawTurnX, rawTurnY);
                if (turnMag > ROTATION_DEADBAND) {
                    targetHeading = new Rotation2d(rawTurnX, rawTurnY);
                    double scaledTurn = MathUtil.applyDeadband(
                        Math.min(turnMag, 1.0), ROTATION_DEADBAND
                    );
                    targetHeadingRateScale = MathUtil.interpolate(0.1, 1.0, scaledTurn);
                    hasTargetHeading = true;
                }
                SmartDashboard.putNumber("Driver/RawFwd", rawFwd);
                SmartDashboard.putNumber("Driver/RawStrafe", rawStrafe);
                SmartDashboard.putNumber("Driver/TargetHeadingDeg", targetHeading.getDegrees());

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
                    double currentSpeed = wasMoving ? speedRate.calculate(0.0) : 0.0;
                    if (currentSpeed <= 1e-3) {
                        wasMoving = false;
                        speedRate.reset(0.0);
                        vx = 0.0;
                        vy = 0.0;
                    } else {
                        vx = currentSpeed * Math.cos(smoothedAngle);
                        vy = currentSpeed * Math.sin(smoothedAngle);
                    }
                }

                if (hasTargetHeading) {
                    return facingDrive.withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(targetHeading)
                        .withMaxAbsRotationalRate(
                            MAX_ROT_RAD_PER_SEC * slow * targetHeadingRateScale
                        );
                }

                return drive.withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(0.0);
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> {
                hasTargetHeading = false;
                wasMoving = false;
                speedRate.reset(0.0);
                return idle;
            }).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on Select (Back) button press.
        joystick.back().onTrue(drivetrain.runOnce(() -> {
            drivetrain.seedFieldCentric();
            hasTargetHeading = false;
        }));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Command driveToPoint(double targetX, double targetY) {
        final var autoDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
            .withDesaturateWheelSpeeds(true);

        return drivetrain.applyRequest(() -> {
            Pose2d pose = drivetrain.getState().Pose;
            double dx = targetX - pose.getX();
            double dy = targetY - pose.getY();
            double dist = Math.hypot(dx, dy);

            double vx = 0.0;
            double vy = 0.0;
            if (dist > 1e-4) {
                double speed = MathUtil.clamp(dist * 2.0, 0.0, 1.0);
                vx = speed * (dx / dist);
                vy = speed * (dy / dist);
            }

            double headingError = MathUtil.angleModulus(-pose.getRotation().getRadians());
            double omega = MathUtil.clamp(headingError * 3.0, -1.5, 1.5);

            return autoDrive.withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(omega);
        })
        .until(() -> {
            Pose2d pose = drivetrain.getState().Pose;
            return Math.hypot(targetX - pose.getX(), targetY - pose.getY()) < 0.04;
        })
        .withTimeout(4.0);
    }

    private Command driveSpinningCircle(double diameterMeters, double durationSeconds, double spinRadPerSec) {
        final var autoDrive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
            .withDesaturateWheelSpeeds(true);

        final Timer timer = new Timer();
        final double radius = diameterMeters / 2.0;
        final double pathOmega = (2.0 * Math.PI) / durationSeconds;

        return Commands.sequence(
            drivetrain.runOnce(() -> {
                drivetrain.resetPose(Pose2d.kZero);
                timer.restart();
            }),
            drivetrain.applyRequest(() -> {
                double t = Math.min(timer.get(), durationSeconds);
                double theta = pathOmega * t;
                double targetX = radius * (1.0 - Math.cos(theta));
                double targetY = radius * Math.sin(theta);

                Pose2d pose = drivetrain.getState().Pose;
                double vx = radius * pathOmega * Math.sin(theta) + 2.0 * (targetX - pose.getX());
                double vy = radius * pathOmega * Math.cos(theta) + 2.0 * (targetY - pose.getY());

                return autoDrive.withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(spinRadPerSec);
            }).until(() -> timer.hasElapsed(durationSeconds))
        );
    }

    private void configureAutoChooser() {
        final var idle = new SwerveRequest.Idle();

        Command squareAuto = Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.resetPose(Pose2d.kZero)),
            driveToPoint(1.0, 0.0),
            driveToPoint(1.0, 1.0),
            driveToPoint(0.0, 1.0),
            driveToPoint(0.0, 0.0),
            drivetrain.applyRequest(() -> idle)
        ).withName("Auto Square");

        Command driveForwardAuto = Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            ).withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        ).withName("Drive Forward");

        Command spinningCircleAuto = Commands.sequence(
            driveSpinningCircle(1.0, 4.0, Math.PI),
            drivetrain.applyRequest(() -> idle)
        ).withName("Spinning Circle");

        autoChooser.setDefaultOption("Auto Square", squareAuto);
        autoChooser.addOption("Spinning Circle", spinningCircleAuto);
        autoChooser.addOption("Drive Forward", driveForwardAuto);
        autoChooser.addOption("Do Nothing", Commands.none().withName("Do Nothing"));
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
