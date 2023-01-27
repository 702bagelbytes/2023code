package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmOutCommand extends CommandBase {

    private ArmSubsystem armSubsystem = new ArmSubsystem();

    public ArmOutCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.setArmSpeed(-1);
    }

    public void end(boolean interrupted) {
        armSubsystem.setArmSpeed(0);
    }

}