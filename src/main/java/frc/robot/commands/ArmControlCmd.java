package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmControlCmd extends CommandBase {
    private final ArmSubsystem arm;
    private final Supplier<Double> input;

    public ArmControlCmd(ArmSubsystem arm, Supplier<Double> input) {
        this.arm = arm;
        addRequirements(arm);
        this.input = input;
    }

    @Override
    public void execute() {
        arm.set(input.get());
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.set(0);
    }
}
