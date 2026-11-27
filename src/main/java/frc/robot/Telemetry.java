package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Telemetry {
    private final double MaxSpeed;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public Telemetry(double maxSpeed) {
        MaxSpeed = maxSpeed;
        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
            SmartDashboard.putData("Module " + i, m_moduleMechanisms[i]);
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Per-module setpoint vs position publishers for AdvantageScope */
    private static final String[] kModuleNames = {"ModFL", "ModFR", "ModBL", "ModBR"};
    private static final int kNumModules = kModuleNames.length;
    private final DoublePublisher[] modSteerSetpointDeg = new DoublePublisher[kNumModules];
    private final DoublePublisher[] modSteerPositionDeg = new DoublePublisher[kNumModules];
    private final DoublePublisher[] modSteerErrorDeg = new DoublePublisher[kNumModules];
    private final DoublePublisher[] modDrivePositionMeters = new DoublePublisher[kNumModules];
    private final DoublePublisher[] modDriveSetpointMPS = new DoublePublisher[kNumModules];
    private final DoublePublisher[] modDriveVelocityMPS = new DoublePublisher[kNumModules];

    {
        for (int i = 0; i < kNumModules; i++) {
            NetworkTable modTable = driveStateTable.getSubTable(kModuleNames[i]);
            modSteerSetpointDeg[i] = modTable.getDoubleTopic("SteerSetpointDeg").publish();
            modSteerPositionDeg[i] = modTable.getDoubleTopic("SteerPositionDeg").publish();
            modSteerErrorDeg[i] = modTable.getDoubleTopic("SteerErrorDeg").publish();
            modDrivePositionMeters[i] = modTable.getDoubleTopic("DrivePositionMeters").publish();
            modDriveSetpointMPS[i] = modTable.getDoubleTopic("DriveSetpointMPS").publish();
            modDriveVelocityMPS[i] = modTable.getDoubleTopic("DriveVelocityMPS").publish();
        }
    }

    /* Robot pose for field positioning */
    private final NetworkTable table = inst.getTable("Pose");
    private final DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher fieldTypePub = table.getStringTopic(".type").publish();

    /* Mechanisms to represent the swerve module states */
    private final Mechanism2d[] m_moduleMechanisms = new Mechanism2d[] {
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
        new Mechanism2d(1, 1),
    };
    /* A direction and length changing ligament for speed representation */
    private final MechanismLigament2d[] m_moduleSpeeds = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[1].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[2].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
        m_moduleMechanisms[3].getRoot("RootSpeed", 0.5, 0.5).append(new MechanismLigament2d("Speed", 0.5, 0)),
    };
    /* A direction changing and length constant ligament for module direction */
    private final MechanismLigament2d[] m_moduleDirections = new MechanismLigament2d[] {
        m_moduleMechanisms[0].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[1].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[2].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
        m_moduleMechanisms[3].getRoot("RootDirection", 0.5, 0.5)
            .append(new MechanismLigament2d("Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
    };

    private final double[] m_poseArray = new double[3];

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeStructArray("DriveState/ModuleStates", SwerveModuleState.struct, state.ModuleStates);
        SignalLogger.writeStructArray("DriveState/ModuleTargets", SwerveModuleState.struct, state.ModuleTargets);
        SignalLogger.writeStructArray("DriveState/ModulePositions", SwerveModulePosition.struct, state.ModulePositions);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
        SignalLogger.writeInteger("DriveState/FailedDaqs", state.FailedDaqs);

        /* Publish and log per-module setpoint vs position for AdvantageScope */
        for (int i = 0; i < kNumModules; i++) {
            if (state.ModuleTargets != null && i < state.ModuleTargets.length &&
                state.ModuleStates != null && i < state.ModuleStates.length) {
                double setpointDeg = MathUtil.inputModulus(state.ModuleTargets[i].angle.getDegrees(), -180.0, 180.0);
                double positionDeg = MathUtil.inputModulus(state.ModuleStates[i].angle.getDegrees(), -180.0, 180.0);
                double errorDeg = MathUtil.inputModulus(setpointDeg - positionDeg, -180.0, 180.0);
                double targetSpeed = state.ModuleTargets[i].speedMetersPerSecond;
                double actualSpeed = state.ModuleStates[i].speedMetersPerSecond;

                modSteerSetpointDeg[i].set(setpointDeg);
                modSteerPositionDeg[i].set(positionDeg);
                modSteerErrorDeg[i].set(errorDeg);
                modDriveSetpointMPS[i].set(targetSpeed);
                modDriveVelocityMPS[i].set(actualSpeed);

                SignalLogger.writeDouble("DriveState/" + kModuleNames[i] + "/SteerSetpointDeg", setpointDeg, "deg");
                SignalLogger.writeDouble("DriveState/" + kModuleNames[i] + "/SteerPositionDeg", positionDeg, "deg");
                SignalLogger.writeDouble("DriveState/" + kModuleNames[i] + "/SteerErrorDeg", errorDeg, "deg");
                SignalLogger.writeDouble("DriveState/" + kModuleNames[i] + "/DriveSetpointMPS", targetSpeed, "mps");
                SignalLogger.writeDouble("DriveState/" + kModuleNames[i] + "/DriveVelocityMPS", actualSpeed, "mps");
            }
            if (state.ModulePositions != null && i < state.ModulePositions.length) {
                double drivePosMeters = state.ModulePositions[i].distanceMeters;
                modDrivePositionMeters[i].set(drivePosMeters);
                SignalLogger.writeDouble("DriveState/" + kModuleNames[i] + "/DrivePositionMeters", drivePosMeters, "m");
            }
        }

        /* Telemeterize the pose to a Field2d */
        fieldTypePub.set("Field2d");

        m_poseArray[0] = state.Pose.getX();
        m_poseArray[1] = state.Pose.getY();
        m_poseArray[2] = state.Pose.getRotation().getDegrees();
        fieldPub.set(m_poseArray);

        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
            m_moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            m_moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            m_moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * MaxSpeed));
        }
    }
}
