package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Float> balanceAngleSupplier;
    private static final PIDController controller = new PIDController(0.05, 0.0, 0.005);

    public BalanceCommand(DriveSubsystem driveSubsystem,
            Supplier<Float> balanceAngleSupplier, double setpoint) {
        this.driveSubsystem = driveSubsystem;
        this.balanceAngleSupplier = balanceAngleSupplier;
        controller.setTolerance(2);
        controller.setSetpoint(setpoint);
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double speed = driveSubsystem.chargeStationClampSpeed(controller.calculate(balanceAngleSupplier.get()));
        SmartDashboard.putData(controller);
        SmartDashboard.putNumber("Auto Speed", speed);
        driveSubsystem.tankDrive(speed, speed);

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Auto Speed", 0);
        driveSubsystem.tankDrive(0, 0);
    }
}
