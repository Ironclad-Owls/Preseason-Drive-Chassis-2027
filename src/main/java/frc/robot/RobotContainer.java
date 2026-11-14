package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * RobotContainer defines subsystem instances, button bindings, and autonomous routines.
 */
public class RobotContainer {

    // Subsystems
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    // Driver Controller (Port 0)
    private final CommandXboxController m_driverController =
        new CommandXboxController(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT);

    // Autonomous Chooser
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();

        // Build PathPlanner auto chooser (falls back to a default command if PathPlanner paths aren't loaded yet)
        SendableChooser<Command> chooser;
        try {
            chooser = AutoBuilder.buildAutoChooser();
        } catch (Exception e) {
            chooser = new SendableChooser<>();
            chooser.setDefaultOption("Do Nothing", Commands.none());
        }
        m_autoChooser = chooser;
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    /**
     * Default command bindings.
     * Left Stick: Translation (X = forward/back, Y = strafe left/right)
     * Right Stick: Rotation (Yaw)
     * Left Bumper: Slow / Precision mode
     */
    private void configureDefaultCommands() {
        m_drivetrain.setDefaultCommand(
            DriveCommands.fieldCentricDrive(
                m_drivetrain,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                m_driverController.leftBumper()
            )
        );
    }

    /**
     * Driver button bindings.
     */
    private void configureButtonBindings() {
        // Y Button: Reset / Zero Gyro Heading to Field-Forward
        m_driverController.y().onTrue(DriveCommands.zeroGyro(m_drivetrain));

        // X Button: Defense "X-Stance" Brake (resists being pushed)
        m_driverController.x().whileTrue(DriveCommands.xBrake(m_drivetrain));

        // Right Bumper: Hold for Robot-Centric Driving (useful for intaking / pit testing)
        m_driverController.rightBumper().whileTrue(
            DriveCommands.robotCentricDrive(
                m_drivetrain,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> -m_driverController.getRightX(),
                m_driverController.leftBumper()
            )
        );

        // D-Pad / POV Quick Heading Snapping
        // POV Up: Snap to 0 degrees (Field Forward)
        m_driverController.povUp().whileTrue(
            DriveCommands.faceAngleDrive(
                m_drivetrain,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                Rotation2d.fromDegrees(0),
                m_driverController.leftBumper()
            )
        );

        // POV Right: Snap to -90 degrees (Field Right)
        m_driverController.povRight().whileTrue(
            DriveCommands.faceAngleDrive(
                m_drivetrain,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                Rotation2d.fromDegrees(-90),
                m_driverController.leftBumper()
            )
        );

        // POV Down: Snap to 180 degrees (Field Backward)
        m_driverController.povDown().whileTrue(
            DriveCommands.faceAngleDrive(
                m_drivetrain,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                Rotation2d.fromDegrees(180),
                m_driverController.leftBumper()
            )
        );

        // POV Left: Snap to 90 degrees (Field Left)
        m_driverController.povLeft().whileTrue(
            DriveCommands.faceAngleDrive(
                m_drivetrain,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                Rotation2d.fromDegrees(90),
                m_driverController.leftBumper()
            )
        );
    }

    /**
     * Pass the autonomous command to the main Robot class.
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    /**
     * Accessor for drivetrain subsystem.
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return m_drivetrain;
    }
}
