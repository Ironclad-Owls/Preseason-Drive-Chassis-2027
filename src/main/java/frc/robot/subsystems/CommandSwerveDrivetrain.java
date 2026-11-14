package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.configs.SwerveConfig;

/**
 * Command-based Swerve Drivetrain subsystem extending CTRE's Phoenix 6 SwerveDrivetrain.
 * Handles:
 *  - High-performance odometry thread
 *  - Telemetry and field pose publishing
 *  - PathPlanner autonomous configuration
 *  - Robot-relative and Field-relative drive controls
 *  - X-brake stance
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

    private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms sim loop
    private Notifier m_simNotifier = null;
    private double m_lastSimTime = 0.0;

    // Requests used for autonomous and standard modes
    private final SwerveRequest.ApplyRobotSpeeds m_robotRelativeRequest =
        new SwerveRequest.ApplyRobotSpeeds();

    private final SwerveRequest.SwerveDriveBrake m_brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    private final SwerveTelemetry m_telemetry =
        new SwerveTelemetry(SwerveConfig.MAX_LINEAR_SPEED_MPS);

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveConstants, odometryUpdateFrequency, modules);
        initDrivetrain();
    }

    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants driveConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(driveConstants, modules);
        initDrivetrain();
    }

    private void initDrivetrain() {
        // Register telemetry callback
        registerTelemetry(m_telemetry::telemeterize);

        // Configure PathPlanner AutoBuilder
        configurePathPlanner();

        // Start simulation thread if in simulation
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Applies a continuous swerve control request to the drivetrain.
     *
     * @param requestSupplier Supplier returning the request to apply
     * @return Command that continuously executes the request
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Sets robot-relative chassis speeds (used primarily by PathPlanner).
     *
     * @param speeds Desired robot-relative ChassisSpeeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        setControl(m_robotRelativeRequest.withSpeeds(speeds));
    }

    /**
     * Gets current robot-relative chassis speeds.
     */
    public ChassisSpeeds getCurrentChassisSpeeds() {
        return getState().Speeds;
    }

    /**
     * Returns current robot pose on the field.
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Resets the robot's current odometry pose.
     *
     * @param pose The new pose
     */
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    /**
     * Zeros the Pigeon 2 gyro heading (makes current orientation forward).
     */
    public void zeroHeading() {
        seedFieldCentric();
    }

    /**
     * Sets the gyro yaw to a specific heading angle.
     *
     * @param heading Heading to set
     */
    public void setHeading(Rotation2d heading) {
        getPigeon2().setYaw(heading.getDegrees());
    }

    /**
     * Gets current robot heading.
     */
    public Rotation2d getHeading() {
        return getState().Pose.getRotation();
    }

    /**
     * Returns a command to lock swerve wheels into an 'X' pattern to resist pushing.
     */
    public Command xBrakeCommand() {
        return applyRequest(() -> m_brakeRequest);
    }

    /**
     * Returns the telemetry object.
     */
    public SwerveTelemetry getTelemetry() {
        return m_telemetry;
    }

    /**
     * Configures PathPlanner AutoBuilder for autonomous path following.
     */
    private void configurePathPlanner() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getCurrentChassisSpeeds,
                this::driveRobotRelative,
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID
                    new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID
                ),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                },
                this
            );
        } catch (Exception e) {
            DriverStation.reportWarning(
                "[CommandSwerveDrivetrain] PathPlanner RobotConfig not found or failed to load: " + e.getMessage()
                + " (Run PathPlanner GUI to create settings.json to enable autonomous paths)",
                false
            );
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        // Run simulation at 200 Hz
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            // Update CTRE simulation model
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(SIM_LOOP_PERIOD);
    }

    @Override
    public void periodic() {
        // Periodic logic if needed
    }
}
