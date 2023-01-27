package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretSpinCommand extends CommandBase {
    SlewRateLimiter filter;
    private WPI_TalonFX talonFX;

    public TurretSpinCommand(WPI_TalonFX talonFX, SlewRateLimiter filter) {

        this.filter = filter;
        this.talonFX = talonFX;

    }

    @Override
    public void execute() {
        talonFX.set(filter.calculate(0.5));
    }

    @Override
    public void end(boolean interrupted) {
        talonFX.set(0);
    }

}
