package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.ArmSubsystem;

public class ArmPIDCommand extends CommandBase {
  ArmSubsystem armSubsystem;
  PIDController ArmPIDController = new PIDController(0.08, 0, 0);

  public ArmPIDCommand(ArmSubsystem armSubsystem, double setpoint) {
    this.armSubsystem = armSubsystem;
    ArmPIDController.setSetpoint(setpoint);
    ArmPIDController.setTolerance(4);
  }

  @Override
  public void initialize() {
    armSubsystem.resetEncoders();

  }

  @Override
  public void execute() {

    double speed = ArmPIDController.calculate(armSubsystem.getEncoderPositionDeg());
    armSubsystem.set(speed);

  }
}