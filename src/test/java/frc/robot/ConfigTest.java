package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import frc.robot.configs.SwerveConfig;

public class ConfigTest {

    @Test
    void testGearRatiosAndGeometry() {
        assertTrue(SwerveConfig.DRIVE_GEAR_RATIO > 0, "Drive gear ratio must be positive");
        assertTrue(SwerveConfig.STEER_GEAR_RATIO > 0, "Steer gear ratio must be positive");
        assertTrue(SwerveConfig.WHEEL_RADIUS_METERS > 0, "Wheel radius must be positive");
        assertTrue(SwerveConfig.TRACK_WIDTH_X_METERS > 0, "Trackwidth X must be positive");
        assertTrue(SwerveConfig.TRACK_WIDTH_Y_METERS > 0, "Trackwidth Y must be positive");
    }

    @Test
    void testCurrentLimits() {
        assertTrue(SwerveConfig.DRIVE_STATOR_CURRENT_LIMIT > 0, "Drive stator limit must be positive");
        assertTrue(SwerveConfig.DRIVE_SUPPLY_CURRENT_LIMIT > 0, "Drive supply limit must be positive");
        assertTrue(SwerveConfig.STEER_STATOR_CURRENT_LIMIT > 0, "Steer stator limit must be positive");
        assertTrue(SwerveConfig.STEER_SUPPLY_CURRENT_LIMIT > 0, "Steer supply limit must be positive");
        assertTrue(SwerveConfig.DRIVE_SUPPLY_CURRENT_LIMIT <= 60.0, "RoboRIO 1 drive supply limit should be <= 60A to avoid brownouts");
    }

    @Test
    void testCanIdsUnique() {
        int[] ids = {
            SwerveConfig.FL_DRIVE_MOTOR_ID, SwerveConfig.FL_STEER_MOTOR_ID, SwerveConfig.FL_CANCODER_ID,
            SwerveConfig.FR_DRIVE_MOTOR_ID, SwerveConfig.FR_STEER_MOTOR_ID, SwerveConfig.FR_CANCODER_ID,
            SwerveConfig.BL_DRIVE_MOTOR_ID, SwerveConfig.BL_STEER_MOTOR_ID, SwerveConfig.BL_CANCODER_ID,
            SwerveConfig.BR_DRIVE_MOTOR_ID, SwerveConfig.BR_STEER_MOTOR_ID, SwerveConfig.BR_CANCODER_ID,
            SwerveConfig.PIGEON2_CAN_ID
        };

        for (int i = 0; i < ids.length; i++) {
            assertTrue(ids[i] >= 1 && ids[i] <= 62, "CAN ID must be in valid range [1, 62]: " + ids[i]);
            for (int j = i + 1; j < ids.length; j++) {
                assertNotEquals(ids[i], ids[j], "CAN IDs must be distinct: ID " + ids[i] + " matches index " + j);
            }
        }
    }
}
