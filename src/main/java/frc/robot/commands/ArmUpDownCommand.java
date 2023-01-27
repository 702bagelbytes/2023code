package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmXYSubsystem;

public class ArmUpDownCommand extends CommandBase {
    private ArmXYSubsystem armSubsystem = new ArmXYSubsystem();
    private Supplier<Double> dataSupplier;
    private SlewRateLimiter filter = new SlewRateLimiter(.5, -.5, .1);

    public ArmUpDownCommand(ArmXYSubsystem armSubsystem, Supplier<Double> dataSupplier) {
        this.armSubsystem = armSubsystem;
        this.dataSupplier = dataSupplier;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        double amtMoved = dataSupplier.get();
        armSubsystem.setArmUpDownSpeed(filter.calculate(amtMoved * 1.5 /* twice as fast */));
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setArmUpDownSpeed(0);
    }

}
