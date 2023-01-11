package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final double driveVal;
    private final double strafeVal;
    private final double turnVal;

    public DriveCmd(
        DriveSubsystem driveSubsystem, double driveVal, double strafeVal, double turnVal
    ) {
        this.driveSubsystem = driveSubsystem;
        this.driveVal = driveVal;
        this.strafeVal = strafeVal;
        this.turnVal = turnVal;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("DriveCmd start");
        driveSubsystem.loggingEnabled = true;
    }

    @Override
    public void execute() {
        driveSubsystem.driveCartesian(driveVal, strafeVal, turnVal, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("DriveCmd end");
        driveSubsystem.loggingEnabled = false;
    }
}
