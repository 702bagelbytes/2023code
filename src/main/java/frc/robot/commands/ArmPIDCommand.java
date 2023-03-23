package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;

public class ArmPIDCommand extends CommandBase {
  ArmSubsystem armSubsystem;

  public ArmPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    armSubsystem.getArmPID().setSetpoint(setpoint);

  }

  @Override
  public boolean isFinished() {
    if (armSubsystem.getArmPID().atSetpoint()) {
      armSubsystem.getTalon().set(0);
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean isInterrupted) {
    armSubsystem.getTalon().set(0);

  }
}