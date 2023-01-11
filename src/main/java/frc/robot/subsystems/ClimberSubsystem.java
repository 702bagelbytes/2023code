package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private WPI_TalonFX right = new WPI_TalonFX(15);
    private WPI_TalonFX left = new WPI_TalonFX(2);
    private Encoder leftEncoder = new Encoder(2, 3); // lol
    private Encoder rightEncoder = new Encoder(
        ClimberConstants.RIGHT_ENCODER_PIN_A, ClimberConstants.RIGHT_ENCODER_PIN_B
    );

    public ClimberSubsystem() {
        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
    }

    private void set(MotorController spark, double val) {
        spark.set(val * ClimberConstants.POWER);
    }

    public void setLeft(double val) {
        set(left, val);
    }

    public void setRight(double val) {
        set(right, -val);
    }

    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    public Encoder getRightEncoder() {
        return rightEncoder;
    }
}
