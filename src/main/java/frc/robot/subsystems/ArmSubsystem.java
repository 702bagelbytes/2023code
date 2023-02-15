package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax raiseSpark = new CANSparkMax(ArmConstants.kRaiseSpark, MotorType.kBrushed);
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(0.7, -0.7, 0.2);
    Encoder encoder = new Encoder(0, 1);

    public ArmSubsystem() {
        raiseSpark.setIdleMode(IdleMode.kBrake);
        // 1 / cpr / gear ratio
        encoder.setDistancePerPulse(1 / 1024.0 / 48.0);
        rateLimiter.reset(0);
    }

    public void set(double value) {
        if (value == 0) {
            rateLimiter.reset(0);
        }
        raiseSpark.set(value == 0 ? value : rateLimiter.calculate(value));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armEncoder", encoder.getDistance());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));
    }
}