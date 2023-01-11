package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class SingleClimberCmd extends CommandBase {
    public enum Side {
        kLeft(0),
        kRight(1);

        private Side(int val) {}
    }

    private final ClimberSubsystem climber;
    private final Side side;
    private final double val;

    public SingleClimberCmd(ClimberSubsystem climber, Side side, double val) {
        this.climber = climber;
        this.side = side;
        this.val = val;
    }

    @Override
    public void execute() {
        switch(side) {
            case kLeft:
                climber.setLeft(val);
                break;
            case kRight:
                climber.setRight(val);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climber.setLeft(0);
        climber.setRight(0);
    }
}
