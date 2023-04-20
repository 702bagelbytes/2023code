// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCoDriverControllerPort = 1;
  }

  public static double inchToMeter(double inches) {
    return inches * .0254;
  }

  public static class DriveConstants {
    public static final int SPARK_FL_ID = 1;
    public static final int TALON_ML_ID = 36;
    public static final int TALON_BL_ID = 35;
    public static final int SPARK_FR_ID = 4;
    public static final int TALON_MR_ID = 16;
    public static final int TALON_BR_ID = 27;

    public static final boolean LEFT_ENCODER_INVERTED = true;
    public static final boolean RIGHT_ENCODER_INVERTED = true;
    public static final double ENCODER_OFFSET = 130;
    public static final double DRIVE_SPEED = 0.75;
    public static final double WHEEL_RADIUS_INCH = 4.0;
    public static final double INCH_TO_METER = 0.0254;

    public static class MotorFeed {
      public static final double KS = 0.05;
      public static final double KV = 0.05;
      public static final double KA = 0.03;
    }

    public static class Ramsete {
      public static final double P = 0.5;
      public static final double I = 0.0;
      public static final double D = 0.1;

    }
  }

  public static class PIDConstants {
    public static final double SCORE_HIGH_SETPOINT = 19.4;
    public static final double PICK_UP_PIECE_SETPOINT = -35;

  }

  public static class TurretConstants {
    public static final int TURRET_ID = 32;
    public static final double kMaxOutput = 0.5;
  }

  public static class TelescopeConstants {
    public static final int EXTENSION_TALON_ID = 15;
    public static final double kTelescopeSpeed = 0.8;
  }

  public static class ArmConstants {
    public static final int ARM_TALON_ID = 10;

    public static final double kMaxArmOutput = 0.4;
    public static final double MAX_UP_DEG = 21;
    public static final boolean REVERSE_LIMIT_TOGGLE = false;
    public static final boolean FORWARD_LIMIT_TOGGLE = false;
  }

  public static class EncoderConstants {
    // 1 / cpr / gear ratio
    public static double BASE = 1;
    public static double PULSES_PER_REVOLUTION = 2048;
    public static double GEARBOX_RATIO = 48;
    public static double SMALL_COG_TO_BIG_COG_1 = 2.3077;
    public static double BIG_COG_TO_BIG_COG_2 = 1.733334;
    public static double DEGREES_IN_A_CIRCLE = 360;

    // this must be measured with an Angle Gauge
    public static double ANGLE_TO_ZERO = -72.5;
  }

  public static class GRABOTRONConstants {
    public static final int kRevPneumaticsHubId = 5;
    public static final int kExtendSolenoid = 0;
    public static final int kRetractSolenoid = 1;
  }
}
