// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public class DriveConstants {
        public static final int TALON_FL_ID = 30; // 1-21-2022 (DEMO BOT)
        public static final int SPARK_ML_ID = 3; // 1-21-2022 (DEMO BOT)
        public static final int TALON_BL_ID = 29; // 1-21-2022 (DEMO BOT)

        public static final int SPARK_FR_ID = 1; // 1-21-2022 (DEMO BOT)
        public static final int TALON_MR_ID = 22; // 1-21-2022 (DEMO BOT)
        public static final int TALON_BR_ID = 21; // 1-21-2022 (DEMO BOT)

        public static final boolean LEFT_ENCODER_INVERTED = false;
        public static final boolean RIGHT_ENCODER_INVERTED = true;

        public static final double DRIVE_SPEED = 1.0;
    }

    public class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int CODRIVER_PORT = 1;
    }
}
