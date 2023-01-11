package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberControlCmd extends CommandBase {
    private ClimberSubsystem climber;
    private Supplier<Double> controlSupplier;
    
    public ClimberControlCmd(ClimberSubsystem climber, Supplier<Double> controlSupplier) {
        this.climber = climber;
        addRequirements(climber);
        this.controlSupplier = controlSupplier;
    }

    @Override
    public void initialize() {
        System.out.println("ClimberControlCmd start");
    }

    @Override
    public void execute() {
        double val = controlSupplier.get();
        climber.setLeft(val);
        climber.setRight(val);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ClimberControlCmd end");
        climber.setLeft(0);
    }
}
