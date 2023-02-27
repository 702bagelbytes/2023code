package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX kRaiseTalonFX = new WPI_TalonFX(Constants.ArmConstants.kRaiseTalonFX);
    // private final SlewRateLimiter rateLimiter = new SlewRateLimiter(0.7, -0.7,
    // 0.0);
    Encoder encoder = new Encoder(0, 1, false, EncodingType.k2X);

    public ArmSubsystem() {
        kRaiseTalonFX.setNeutralMode(NeutralMode.Brake);
        // 1 / cpr / gear ratio
        // encoder.setDistancePerPulse(1 / 2048.0 / 48.0);
        encoder.reset();
        // rateLimiter.reset(0);

    }
    
    public Encoder getEncoder() {
        return encoder;
    }

    public void set(double value) {
        // if (value == 0) {
        // rateLimiter.reset(0);
        // }
        // var valSend = value == 0 ? value : rateLimiter.calculate(value);
        kRaiseTalonFX.set(value * Constants.ArmConstants.kMaxArmOutput);
        // SmartDashboard.putNumber("ArmValSend", valSend);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armEncoder", encoder.getDistance());
    }
    
    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));
    }
}