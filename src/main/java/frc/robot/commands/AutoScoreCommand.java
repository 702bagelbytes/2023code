package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GRABOTRONSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class AutoScoreCommand extends CommandBase {
  ArmSubsystem armSubsystem;
  GRABOTRONSubsystem grabotronSubsystem;
  TelescopeSubsystem telescopeSubsystem;

  public AutoScoreCommand(ArmSubsystem armSubsystem, GRABOTRONSubsystem grabotronSubsystem,
      TelescopeSubsystem telescopeSubsystem) {
    this.armSubsystem = armSubsystem;
    this.grabotronSubsystem = grabotronSubsystem;
    this.telescopeSubsystem = telescopeSubsystem;
  }

  @Override
  public void initialize() {
    armSubsystem.resetEncoders();
    telescopeSubsystem.resetEncoders();
  }

  @Override
  public void execute() {

  }

}