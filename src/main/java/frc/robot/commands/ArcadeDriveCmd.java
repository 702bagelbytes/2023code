// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** */
public class ArcadeDriveCmd extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> driveFunction, strafeFunction, turnFunction;
  private final Supplier<Boolean> gryoEnabledFunction;

  /**
   * Creates a new ArcadeDriveCmd.
   *
   * @param subsystem The subsystem used by this command.
   * @param 
   */
  public ArcadeDriveCmd(
    DriveSubsystem driveSubsystem, Supplier<Double> driveFunction, 
    Supplier<Double> strafeFunction, Supplier<Double> turnFunction, 
    Supplier<Boolean> gyroEnabledFunction
  ) {
    this.driveSubsystem = driveSubsystem;
    this.driveFunction = driveFunction;
    this.strafeFunction = strafeFunction;
    this.turnFunction = turnFunction;
    this.gryoEnabledFunction = gyroEnabledFunction;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArcadeDriveCmd started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.driveCartesian(
      driveFunction.get(), strafeFunction.get(), turnFunction.get(), 
      gryoEnabledFunction.get()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
