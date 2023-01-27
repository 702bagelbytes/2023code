package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstantsDemobot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ArmXYSubsystem extends SubsystemBase {

    private WPI_TalonSRX armUpDown = new WPI_TalonSRX(Constants.DriveConstantsDemobot.ARMUPDOWN_ID);

    public ArmXYSubsystem() {
        armUpDown.setNeutralMode(NeutralMode.Brake);

    }

    public void setArmUpDownSpeed(double val) {
        armUpDown.set(val * DriveConstantsDemobot.ARMUPDOWN_SPEED);
    }
}
