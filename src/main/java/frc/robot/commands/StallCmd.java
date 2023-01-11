package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class StallCmd extends CommandBase {
    private final ArmSubsystem arm;

    public StallCmd(ArmSubsystem arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.set(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        arm.set(0);
    }
}
