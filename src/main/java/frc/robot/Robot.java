// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private static final String[] kModuleNames = {"ModFL", "ModFR", "ModBL", "ModBR"};
    private final RobotContainer m_robotContainer;
    private final PowerDistribution m_pdh = new PowerDistribution();
    private double m_minBatteryVoltage = RobotController.getBatteryVoltage();
    private double m_minPdhVoltage = m_pdh.getVoltage();
    private double m_maxPdhTotalAmps = 0.0;
    private boolean m_wasEnabled = false;
    private final double[] m_minDriveVolts = new double[4];
    private final double[] m_maxDriveAmps = new double[4];
    private final double[] m_minSteerVolts = new double[4];
    private final double[] m_maxSteerAmps = new double[4];
    private final Alert m_lowVoltageWarningAlert =
        new Alert("Battery < 7.5V: Change Battery", AlertType.kWarning);
    private final Alert m_criticalVoltageErrorAlert =
        new Alert("Battery < 6.9V: Brownout", AlertType.kError);
    private final Alert m_highMotorCurrentAlert =
        new Alert("Motor current > 60A", AlertType.kWarning);

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        double batteryVoltage = RobotController.getBatteryVoltage();
        double pdhVoltage = m_pdh.getVoltage();
        double pdhTotalAmps = m_pdh.getTotalCurrent();
        boolean isEnabled = DriverStation.isEnabled();
        var modules = m_robotContainer.drivetrain.getModules();
        if (isEnabled && !m_wasEnabled) {
            m_minBatteryVoltage = batteryVoltage;
            m_minPdhVoltage = pdhVoltage;
            m_maxPdhTotalAmps = pdhTotalAmps;
            for (int i = 0; i < modules.length; i++) {
                m_minDriveVolts[i] = modules[i].getDriveMotor().getSupplyVoltage().getValueAsDouble();
                m_maxDriveAmps[i] = 0.0;
                m_minSteerVolts[i] = modules[i].getSteerMotor().getSupplyVoltage().getValueAsDouble();
                m_maxSteerAmps[i] = 0.0;
            }
        } else if (isEnabled) {
            m_minBatteryVoltage = Math.min(m_minBatteryVoltage, batteryVoltage);
            m_minPdhVoltage = Math.min(m_minPdhVoltage, pdhVoltage);
            m_maxPdhTotalAmps = Math.max(m_maxPdhTotalAmps, pdhTotalAmps);
            for (int i = 0; i < modules.length; i++) {
                double driveVolts = modules[i].getDriveMotor().getSupplyVoltage().getValueAsDouble();
                double driveAmps = Math.abs(modules[i].getDriveMotor().getSupplyCurrent().getValueAsDouble());
                double steerVolts = modules[i].getSteerMotor().getSupplyVoltage().getValueAsDouble();
                double steerAmps = Math.abs(modules[i].getSteerMotor().getSupplyCurrent().getValueAsDouble());

                m_minDriveVolts[i] = Math.min(m_minDriveVolts[i], driveVolts);
                m_maxDriveAmps[i] = Math.max(m_maxDriveAmps[i], driveAmps);
                m_minSteerVolts[i] = Math.min(m_minSteerVolts[i], steerVolts);
                m_maxSteerAmps[i] = Math.max(m_maxSteerAmps[i], steerAmps);
            }
        }
        m_wasEnabled = isEnabled;

        SmartDashboard.putNumber("Battery/Voltage", round1(batteryVoltage));
        SmartDashboard.putNumber("Battery/MinVoltageSinceEnabled", round1(m_minBatteryVoltage));
        boolean criticalVoltage = m_minBatteryVoltage < 6.9;
        m_criticalVoltageErrorAlert.set(criticalVoltage);
        m_lowVoltageWarningAlert.set(!criticalVoltage && m_minBatteryVoltage < 7.5);
        SmartDashboard.putNumber("PDH/Voltage", round1(pdhVoltage));
        SmartDashboard.putNumber("PDH/MinVoltage", round1(m_minPdhVoltage));
        SmartDashboard.putNumber("PDH/TotalAmps", round1(pdhTotalAmps));
        SmartDashboard.putNumber("PDH/MaxTotalAmps", round1(m_maxPdhTotalAmps));

        StringBuilder highCurrentMotors = new StringBuilder();
        for (int i = 0; i < modules.length; i++) {
            String prefix = "DrivetrainDiag/" + kModuleNames[i] + "/";
            double driveImpedance = calcImpedance(m_minPdhVoltage, m_minDriveVolts[i], m_maxDriveAmps[i]);
            double steerImpedance = calcImpedance(m_minPdhVoltage, m_minSteerVolts[i], m_maxSteerAmps[i]);
            SmartDashboard.putNumber(prefix + "DriveMinVolts", round1(m_minDriveVolts[i]));
            SmartDashboard.putNumber(prefix + "DriveMaxAmps", round1(m_maxDriveAmps[i]));
            SmartDashboard.putNumber(prefix + "DriveImpedanceOhms", round4(driveImpedance));
            SmartDashboard.putNumber(prefix + "SteerMinVolts", round1(m_minSteerVolts[i]));
            SmartDashboard.putNumber(prefix + "SteerMaxAmps", round1(m_maxSteerAmps[i]));
            SmartDashboard.putNumber(prefix + "SteerImpedanceOhms", round4(steerImpedance));

            if (m_maxDriveAmps[i] > 60.0) {
                if (highCurrentMotors.length() > 0) highCurrentMotors.append(", ");
                highCurrentMotors.append(kModuleNames[i]).append(" Drive");
            }
            if (m_maxSteerAmps[i] > 60.0) {
                if (highCurrentMotors.length() > 0) highCurrentMotors.append(", ");
                highCurrentMotors.append(kModuleNames[i]).append(" Steer");
            }
        }
        boolean hasHighCurrent = highCurrentMotors.length() > 0;
        if (hasHighCurrent) {
            m_highMotorCurrentAlert.setText("Motor current > 60A: " + highCurrentMotors);
        }
        m_highMotorCurrentAlert.set(hasHighCurrent);

        double headingDeg = MathUtil.inputModulus(
            -m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees() + 180.0,
            0.0, 360.0
        );
        SmartDashboard.putNumber("Robot/HeadingDeg", round1(headingDeg));
    }

    private static double calcImpedance(double minPdhVolts, double minMotorVolts, double maxAmps) {
        return maxAmps > 1e-6 ? Math.abs(minPdhVolts - minMotorVolts) / maxAmps : 0.0;
    }

    private static double round1(double value) {
        return Math.round(value * 10.0) / 10.0;
    }

    private static double round4(double value) {
        return Math.round(value * 10000.0) / 10000.0;
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
