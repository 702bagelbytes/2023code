package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakeCommand.IntakeSubsytem;

public class InnerIntakeSubsystem extends SubsystemBase implements IntakeSubsytem {
    private final WPI_TalonSRX talon = new WPI_TalonSRX(IntakeConstants.INNER_TALON_ID);

    public InnerIntakeSubsystem() {
        talon.setNeutralMode(NeutralMode.Brake);
    }

    public void intakeSet(double val) {
        System.out.println("Set inner intake " + val);
        talon.set(val * IntakeConstants.INNER_POWER);
    }
}
