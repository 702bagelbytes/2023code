package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeCommand extends CommandBase {
    public interface IntakeSubsytem extends Subsystem {
        public void intakeSet(double val);
    }

    private final IntakeSubsytem intake;
    private final int direction;

    public IntakeCommand(IntakeSubsytem intake, int direction) {
        addRequirements(intake);
        this.intake = intake;
        this.direction = direction;
    } 

    @Override
    public void initialize() {
        intake.intakeSet(direction);
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeSet(0.0);
    }
}
