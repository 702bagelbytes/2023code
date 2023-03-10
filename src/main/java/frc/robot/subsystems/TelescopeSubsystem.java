package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopeConstants;

public class TelescopeSubsystem extends SubsystemBase {
    private final WPI_TalonFX extensionTalon = new WPI_TalonFX(TelescopeConstants.EXTENSION_TALON_ID);
    /**
     * At least that's what Adam said...
     */
    private final static double GEARBOX_RATIO = 36;

    SlewRateLimiter limiter = new SlewRateLimiter(1.0, -1.0, 0.0);

    public TelescopeSubsystem() {
        extensionTalon.setNeutralMode(NeutralMode.Brake);

    }

    public void resetEncoders() {
        extensionTalon.setSelectedSensorPosition(0);
    }

    public void set(double value) {
        extensionTalon.set(.75 * value);
    }

    /**
     * 
     * @return the sensor position :)
     * @see https://v5.docs.ctr-electronics.com/en/stable/ch14_MCSensor.html
     */
    public double getEncoderPosition() {
        return extensionTalon.getSelectedSensorPosition() / 2048d / TelescopeSubsystem.GEARBOX_RATIO;
    }

    public Command resetEncodersCommand() {
        return this.runOnce(() -> resetEncoders());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.startEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Telescope Dist", this.getEncoderPosition());
    }
}
