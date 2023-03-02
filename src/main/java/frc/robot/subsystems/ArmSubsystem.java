package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX kRaiseTalonFX = new WPI_TalonFX(Constants.ArmConstants.kRaiseTalonFX);
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(4.0);

    public ArmSubsystem() {
        kRaiseTalonFX.setNeutralMode(NeutralMode.Brake);
        kRaiseTalonFX.setSelectedSensorPosition(0);
        // rateLimiter.reset(0);

    }

    public double getEncoderPositionDeg() {
        // 1 / cpr / gear ratio
        final double BASE = 1;
        final double PULSES_PER_REVOLUTION = 2048;
        final double GEARBOX_RATIO = 48;
        final double SMALL_COG_TO_BIG_COG_1 = 4;
        final double BIG_COG_TO_BIG_COG_2 = 4;
        final double DEGREES_IN_A_CIRCLE = 360;

        // this must be measured with an Angle Gauge
        final double ANGLE_TO_ZERO = -75;

        return kRaiseTalonFX.getSelectedSensorPosition()
                / BASE
                / PULSES_PER_REVOLUTION
                / GEARBOX_RATIO
                / SMALL_COG_TO_BIG_COG_1
                / BIG_COG_TO_BIG_COG_2
                * DEGREES_IN_A_CIRCLE
                + ANGLE_TO_ZERO;
    }

    private boolean willRateLimit = false;

    public static final double MAX_UP_DEG = 15;
    public static final double MAX_DOWN_DEG = -70;

    /**
     * Set the speed of the motor. In other words, move it!
     * Don't forget to call this method with zero to stop the motor.
     */
    public void set(double value) {
        double calculated = rateLimiter.calculate(value * Constants.ArmConstants.kMaxArmOutput);

        if (willRateLimit) {
            if (getEncoderPositionDeg() > MAX_UP_DEG && calculated > 0) {
                calculated = 0;
            }

            if (getEncoderPositionDeg() < MAX_DOWN_DEG && calculated < 0) {
                calculated = 0;
            }
        }

        kRaiseTalonFX.set(calculated);

    }

    /**
     * Tried to create a command to bring the arm into its
     * usable range after resetting the encoder value.
     * 
     * @deprecated DO NOT RUN!!!! NOT TESTED
     * @return a command for the {@link SmartDashboard}
     */
    // @Deprecated(since = "3/1/2022")
    // public Command x____Command() {
    // return Commands.runOnce(() -> {
    // while (getEncoderPositionDeg() > MAX_DOWN_DEG) {
    // kRaiseTalonFX.set(.2);
    // }

    // kRaiseTalonFX.set(0);
    // toggleEncoderLimiting();

    // });
    // }

    /**
     * Command to be used in the SmartDashboard.
     * 
     * <b>MUST</b> be clicked:
     * <ol>
     * <li>At the start of the match</li>
     * <li>When the arm is at its lowest point</li>
     * <li>When the arm is not telescoped</li>
     * </ol>
     */
    public Command zeroTheCounter() {
        return Commands.runOnce(() -> this.kRaiseTalonFX.setSelectedSensorPosition(0));
    }

    public Command toggleEncoderLimiting() {
        return Commands.runOnce(() -> willRateLimit = !willRateLimit);
    }

    public boolean isRateLimiting() {
        return willRateLimit;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", this.getEncoderPositionDeg());
        SmartDashboard.putBoolean("Arm Limits", this.isRateLimiting());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));
    }
}