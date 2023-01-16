package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private float zeroValue;
    private Supplier<Float> balanceAngleSupplier;

    public BalanceCommand(DriveSubsystem driveSubsystem, Supplier<Float> balanceAngleSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.zeroValue = 0;
        this.balanceAngleSupplier = balanceAngleSupplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        this.zeroValue = balanceAngleSupplier.get();
    }

    private int bangBangHeading() {
        double angle = balanceAngleSupplier.get();
        if (angle < zeroValue) {
            return -1;
        }
        return 1;
    }

    @Override
    public void execute() {
        double speed = bangBangHeading() * 0.5;
        driveSubsystem.tankDrive(speed, speed);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.tankDrive(0, 0);
    }
}
