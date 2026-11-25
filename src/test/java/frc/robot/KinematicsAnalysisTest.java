package frc.robot;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class KinematicsAnalysisTest {
    @Test
    public void analyzeRapidSteeringKinematics() {
        // Module positions from TunerConstants (12.25 inches = 0.31115 m)
        double d = 0.31115;
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(d, d),   // FL
            new Translation2d(d, -d),  // FR
            new Translation2d(-d, d),  // BL
            new Translation2d(-d, -d)  // BR
        );

        System.out.println("=== SCENARIO 1: Pure rapid turn command changes (+1.5 rad/s to -1.5 rad/s) ===");
        ChassisSpeeds turnPos = new ChassisSpeeds(0, 0, 1.5);
        ChassisSpeeds turnNeg = new ChassisSpeeds(0, 0, -1.5);
        SwerveModuleState[] statesPos = kinematics.toSwerveModuleStates(turnPos);
        SwerveModuleState[] statesNeg = kinematics.toSwerveModuleStates(turnNeg);

        for (int i = 0; i < 4; i++) {
            System.out.printf("Module %d: Pos Angle=%.1f deg, Speed=%.2f m/s | Neg Angle=%.1f deg, Speed=%.2f m/s\n",
                i, statesPos[i].angle.getDegrees(), statesPos[i].speedMetersPerSecond,
                statesNeg[i].angle.getDegrees(), statesNeg[i].speedMetersPerSecond);
            // Check optimization
            SwerveModuleState optimized = SwerveModuleState.optimize(statesNeg[i], statesPos[i].angle);
            System.out.printf("   Optimized to avoid rotation: Angle=%.1f deg, Speed=%.2f m/s\n",
                optimized.angle.getDegrees(), optimized.speedMetersPerSecond);
        }

        System.out.println("\n=== SCENARIO 2: Slow driving forward (0.3 m/s) with rapid steering (-1.5 to +1.5) ===");
        for (double omega = -1.5; omega <= 1.5; omega += 0.5) {
            ChassisSpeeds combo = new ChassisSpeeds(0.3, 0, omega);
            SwerveModuleState[] st = kinematics.toSwerveModuleStates(combo);
            System.out.printf("Omega = %+4.1f rad/s -> FL Angle: %6.1f deg, Speed: %.2f m/s | FR Angle: %6.1f deg, Speed: %.2f m/s\n",
                omega, st[0].angle.getDegrees(), st[0].speedMetersPerSecond,
                st[1].angle.getDegrees(), st[1].speedMetersPerSecond);
        }
    }
}
