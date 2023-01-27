package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmInCommand extends CommandBase {

    private ArmSubsystem armSubsystem = new ArmSubsystem();

    public ArmInCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.setArmSpeed(1);
    }

    public void end(boolean interrupted) {
        armSubsystem.setArmSpeed(0);
    }

}
