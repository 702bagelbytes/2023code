package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstantsDemobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmSubsystem extends SubsystemBase {

    private WPI_TalonSRX talonSRX = new WPI_TalonSRX(Constants.DriveConstantsDemobot.ARM_ID);

    public ArmSubsystem() {
        talonSRX.setNeutralMode(NeutralMode.Brake);
    }

    public void setArmSpeed(double val) {
        talonSRX.set(val * DriveConstantsDemobot.ARM_SPEED);
    }

}
