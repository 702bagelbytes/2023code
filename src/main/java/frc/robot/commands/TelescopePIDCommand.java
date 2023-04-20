package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class TelescopePIDCommand extends CommandBase {
    TelescopeSubsystem telescopeSubsystem;
    PIDController telescopePIDController = new PIDController(3, 0, .005);//used to be one but they changed the gearbox so test it

    public TelescopePIDCommand(TelescopeSubsystem telescopeSubsystem, double setpoint) {
        this.telescopeSubsystem = telescopeSubsystem;
        telescopePIDController.setSetpoint(setpoint);
        telescopePIDController.setTolerance(0.2); //maybe change this to 0.1 or lower?
        addRequirements(telescopeSubsystem);
    }

    @Override
    public void execute() {
        double speed = telescopePIDController.calculate(telescopeSubsystem.getEncoderPosition());
        telescopeSubsystem.set(speed);
    }

    @Override
    public boolean isFinished() {
        return telescopePIDController.atSetpoint();
    }

    @Override
    public void end(boolean isInterrupted) {
        telescopeSubsystem.set(0);
    }
}
