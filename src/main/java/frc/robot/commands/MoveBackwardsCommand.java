package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MoveBackwardsCommand extends CommandBase {
    DriveSubsystem driveSubsystem;

    public MoveBackwardsCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.tankDrive(-0.715, -0.75);
    }

    public void end() {
        driveSubsystem.tankDrive(0, 0);
    }

}