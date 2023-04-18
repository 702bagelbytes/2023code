package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;

public class TurretPIDCommand extends CommandBase {

    TurretSubsystem turretSubsystem;
    PIDController turretPIDController = new PIDController(0.005, 0, 0);

/* 
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
DO NOT USE WITHOUT PROPER TESTING!!!!!!
 */

    public TurretPIDCommand(TurretSubsystem turretSubsystem, double setpoint) {
        this.turretSubsystem = turretSubsystem;
        turretPIDController.setSetpoint(setpoint);
        turretPIDController.setTolerance(2);
        addRequirements(turretSubsystem);

    }

    @Override
    public void execute() {
        double speed = turretPIDController.calculate(turretSubsystem.getRotationsAsDeg());
        turretSubsystem.set(speed);

    }

    @Override
    public boolean isFinished() {
        return turretPIDController.atSetpoint();
    }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.set(0);
    }

}
