package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopePIDCommand extends CommandBase {
    TelescopeSubsystem telescopeSubsystem;
    PIDController telescopePIDController = new PIDController(0.005, 0, 0);

    public TelescopePIDCommand(TelescopeSubsystem telescopeSubsystem, double setpoint) {
        this.telescopeSubsystem = telescopeSubsystem;
        telescopePIDController.setSetpoint(setpoint);
        telescopePIDController.setTolerance(0.2);
    }

    @Override
    public void execute() {
        double speed = telescopePIDController.calculate(telescopeSubsystem.getEncoderPosition());
        telescopeSubsystem.set(speed);
    }

    public boolean isFinished() {
        return telescopePIDController.atSetpoint();
    }
}
