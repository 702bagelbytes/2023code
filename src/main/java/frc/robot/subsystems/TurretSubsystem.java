package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {
    private final WPI_TalonFX talon = new WPI_TalonFX(Constants.TurretConstants.TURRET_ID);
    SlewRateLimiter rateLimiter = new SlewRateLimiter(0.5, -0.5, 0.0);
    LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    // // private final Encoder encoder = new Encoder(0, 0);

    public TurretSubsystem() {
        talon.setNeutralMode(NeutralMode.Brake);
    }

    public void set(double value) {
        talon.set(filter.calculate(value / 6));
    }

    /**
     * turret sprockets have a 14:1 gear ratio.
     * 
     * @return the number of rotations
     */
    public double getRotationsAsDeg() {
        return talon.getSelectedSensorPosition() / 2048D / 140D / 12D * 360;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Encoder Value", getRotationsAsDeg());
    }

    public Command runCmd(DoubleSupplier input) {

        Runnable onTick = () -> {
            double speed = filter.calculate(input.getAsDouble() / 4);

            this.set(speed);
        };

        Runnable end = () -> this.set(0.0);

        return this.runEnd(onTick, end);

    }
}
