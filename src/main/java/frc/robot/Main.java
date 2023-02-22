// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.Map;

import org.yaml.snakeyaml.Yaml;

import edu.wpi.first.wpilibj.RobotBase;
import lombok.Data;

/**
 * Do NOT add any static variables to this class, or any initialization at all.
 * Unless you know what
 * you are doing, do not modify this file except to change the parameter class
 * to the startRobot
 * call.
 */
public final class Main {
  @Data
  public static class Operator {
    public int driver, codriver;
  }

  @Data
  public static class Drive {
    public double speed, wheel_radius, inch_to_meter;

    @Data
    public static class MotorIds {
      public int spark_front, talon_middle, talon_back;
    }

    public MotorIds left, right;
  }

  @Data
  public static class Encoders {
    public boolean invert_left, invert_right;
  }

  @Data
  public static class MotorFeed {
    public double ks, kv, ka;
  }

  @Data
  public static class IdAndSpeed {
    public int talon_id;
    public double max_output;
  }

  @Data
  public static class Grabotron {
    public int rev_pneumatics_hub_id, extend_solenoid, retract_solenoid;
  }

  @Data
  public static class PID {
    public double p, i, d;
  }

  @Data
  public static class Constants {
    public Operator operator;
    public Drive drive;
    public Encoders encoders;
    public MotorFeed motor_feed;
    public IdAndSpeed turret;
    public IdAndSpeed telescope;
    public IdAndSpeed arm;
    public Grabotron grabotron;
    public PID ramsete;
  }

  public static Main.Constants CONFIG;
  // static {
  //   Yaml yaml = new Yaml();
  //   InputStream configFile;
  //   try {
  //     configFile = new FileInputStream("src/main/java/frc/robot/constants.yaml");
  //     CONFIG = yaml.loadAs(configFile, Main.Constants.class);
  //   } catch (FileNotFoundException e) {
  //     System.exit(1);
  //   }

  // }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>
   * If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    // System.out.println(CONFIG);
    RobotBase.startRobot(Robot::new);
  }
}
