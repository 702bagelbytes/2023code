package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;

public class ArmPIDCommand extends CommandBase {
  ArmSubsystem armSubsystem;
  private PIDController armPIDController = new PIDController(0.05, 0, 0.01);

  public ArmPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
    armPIDController.setSetpoint(setpoint);
    armPIDController.setTolerance(5);

  }

  @Override
  public void execute() {
    double speed = armPIDController.calculate(armSubsystem.getEncoderPositionDeg());

    armSubsystem.set(speed);

  }

  @Override
  public boolean isFinished() {
    return armPIDController.atSetpoint();
  }

  @Override
  public void end(boolean isInterrupted) {
    armSubsystem.set(0);

  }
}