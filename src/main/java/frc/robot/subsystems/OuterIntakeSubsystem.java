package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCommand.IntakeSubsytem;

public class OuterIntakeSubsystem extends SubsystemBase implements IntakeSubsytem {
    private final WPI_TalonFX talon = new WPI_TalonFX(IntakeConstants.OUTER_TALON_ID);

    public OuterIntakeSubsystem() {
        talon.setNeutralMode(NeutralMode.Brake);
    }

    public void intakeSet(double val) {
        System.out.println("Set outer intake " + val);
        talon.set(val * IntakeConstants.OUTER_POWER);
    }
}
