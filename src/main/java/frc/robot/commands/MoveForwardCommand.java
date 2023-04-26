
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class MoveForwardCommand extends CommandBase {
    DriveSubsystem driveSubsystem;

    public MoveForwardCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.arcadeDrive(1, 0); // 0.735, 0.75
    }

    public void end() {
        driveSubsystem.arcadeDrive(0, 0);
        
    }

}