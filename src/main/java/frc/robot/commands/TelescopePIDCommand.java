package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopePIDCommand extends CommandBase {
    TelescopeSubsystem telescopeSubsystem;
    PIDController telescopePIDController = new PIDController(0, 0, 0);

    public TelescopePIDCommand(TelescopeSubsystem telescopeSubsystem, double setpoint) {
        this.telescopeSubsystem = telescopeSubsystem;
        telescopePIDController.setSetpoint(setpoint);
    }

    @Override
    public void initialize() {
        telescopeSubsystem.resetEncoders();

    }

    @Override
    public void execute() {
        double speed = telescopePIDController.calculate(telescopeSubsystem.getEncoderPosition());
        telescopeSubsystem.set(speed);
    }
}
