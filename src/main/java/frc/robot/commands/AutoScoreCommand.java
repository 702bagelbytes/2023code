package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GRABOTRONSubsystem;

public class AutoScoreCommand extends CommandBase {
  ArmSubsystem armSubsystem;
  GRABOTRONSubsystem grabotronSubsystem;
  PIDController pidController = new PIDController(0, 0, 0);

  public AutoScoreCommand(ArmSubsystem armSubsystem, GRABOTRONSubsystem grabotronSubsystem) {
    this.armSubsystem = armSubsystem;
    this.grabotronSubsystem = grabotronSubsystem;
  }

  @Override
  public void initialize() {
    // armSubsystem.getEncoder().reset();

  }

  @Override
  public void execute() {

  }

}