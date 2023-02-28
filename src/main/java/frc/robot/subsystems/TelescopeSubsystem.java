package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase {
    private final WPI_TalonFX extensionTalon = new WPI_TalonFX(TelescopeConstants.kExtensionTalonFX);
    SlewRateLimiter limiter = new SlewRateLimiter(1.0, -1.0, 0.0);

    public TelescopeSubsystem() {
        extensionTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void set(double value) {
        extensionTalon.set(TalonFXControlMode.PercentOutput, value);
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.startEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));
    }
}
