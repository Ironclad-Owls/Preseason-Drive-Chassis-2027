package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Telemetry publisher for the Swerve Drivetrain.
 * Publishes odometry pose to Field2d and module telemetry to SmartDashboard.
 */
public class SwerveTelemetry {
    private final double maxSpeed;

    private final Field2d m_field = new Field2d();

    // Mechanism2d visualizations for 4 swerve modules
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    private final MechanismRoot2d[] m_moduleRoots = new MechanismRoot2d[] {
        m_moduleMechanisms[0].getRoot("Root", 0.5, 0.5),
        m_moduleMechanisms[1].getRoot("Root", 0.5, 0.5),
        m_moduleMechanisms[2].getRoot("Root", 0.5, 0.5),
        m_moduleMechanisms[3].getRoot("Root", 0.5, 0.5),
    };
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleRoots[0].append(new MechanismLigament2d("Speed", 0.5, 0, 4, new Color8Bit(Color.kBlue))),
        m_moduleRoots[1].append(new MechanismLigament2d("Speed", 0.5, 0, 4, new Color8Bit(Color.kBlue))),
        m_moduleRoots[2].append(new MechanismLigament2d("Speed", 0.5, 0, 4, new Color8Bit(Color.kBlue))),
        m_moduleRoots[3].append(new MechanismLigament2d("Speed", 0.5, 0, 4, new Color8Bit(Color.kBlue))),
    };
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleRoots[0].append(new MechanismLigament2d("Direction", 0.3, 0, 2, new Color8Bit(Color.kRed))),
        m_moduleRoots[1].append(new MechanismLigament2d("Direction", 0.3, 0, 2, new Color8Bit(Color.kRed))),
        m_moduleRoots[2].append(new MechanismLigament2d("Direction", 0.3, 0, 2, new Color8Bit(Color.kRed))),
        m_moduleRoots[3].append(new MechanismLigament2d("Direction", 0.3, 0, 2, new Color8Bit(Color.kRed))),
    };

    public SwerveTelemetry(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Accepts the latest SwerveDriveState and pushes telemetry to dashboards.
     */
    public void telemeterize(SwerveDriveState state) {
        // Robot Pose
        Pose2d pose = state.Pose;
        m_field.setRobotPose(pose);

        SmartDashboard.putNumber("Swerve/PoseX_m", pose.getX());
        SmartDashboard.putNumber("Swerve/PoseY_m", pose.getY());
        SmartDashboard.putNumber("Swerve/Heading_deg", pose.getRotation().getDegrees());

        // Chassis Speeds
        SmartDashboard.putNumber("Swerve/Vx_mps", state.Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Vy_mps", state.Speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Omega_radps", state.Speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Swerve/OdometryPeriod_s", state.OdometryPeriod);

        // Module states
        for (int i = 0; i < 4; ++i) {
            SwerveModuleState moduleState = state.ModuleStates[i];
            SwerveModuleState targetState = state.ModuleTargets[i];

            SmartDashboard.putNumber("Swerve/Mod" + i + "/ActualSpeed_mps", moduleState.speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Mod" + i + "/ActualAngle_deg", moduleState.angle.getDegrees());
            SmartDashboard.putNumber("Swerve/Mod" + i + "/TargetSpeed_mps", targetState.speedMetersPerSecond);
            SmartDashboard.putNumber("Swerve/Mod" + i + "/TargetAngle_deg", targetState.angle.getDegrees());

            // Update Mechanism2d visualizers
            m_moduleDirections[i].setAngle(moduleState.angle.getDegrees());
            m_moduleSpeeds[i].setAngle(targetState.angle.getDegrees());
            m_moduleSpeeds[i].setLength(targetState.speedMetersPerSecond / (2.0 * maxSpeed));
        }
    }

    public Field2d getField() {
        return m_field;
    }
}
