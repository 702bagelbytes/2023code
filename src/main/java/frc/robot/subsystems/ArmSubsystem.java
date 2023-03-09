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
        kRaiseTalonFX.configReverseSoftLimitThreshold(MAX_DOWN_DEG);
        kRaiseTalonFX.configForwardSoftLimitThreshold(MAX_UP_DEG);
        kRaiseTalonFX.configReverseSoftLimitEnable(REVERSE_LIMIT_TOGGLE);
        kRaiseTalonFX.configForwardSoftLimitEnable(FORWARD_LIMIT_TOGGLE);

    }

    public void resetEncoders() {
        kRaiseTalonFX.setSelectedSensorPosition(0);
    }

    public Command resetEncodersCommand() {

        return this.runOnce(() -> resetEncoders());
    }

    // 1 / cpr / gear ratio
    final static double BASE = 1;
    final static double PULSES_PER_REVOLUTION = 2048;
    final static double GEARBOX_RATIO = 48;
    final static double SMALL_COG_TO_BIG_COG_1 = 4;
    final static double BIG_COG_TO_BIG_COG_2 = 4;
    final static double DEGREES_IN_A_CIRCLE = 360;

    // this must be measured with an Angle Gauge
    final static double ANGLE_TO_ZERO = -72.5;

    public double getEncoderPositionDeg() {

        return degFromTicks(kRaiseTalonFX.getSelectedSensorPosition());
    }

    // private boolean willRateLimit = false;

    public static final double MAX_UP_DEG = 15;
    public static final double MAX_DOWN_DEG = -70;
    public static final boolean REVERSE_LIMIT_TOGGLE = false;
    public static final boolean FORWARD_LIMIT_TOGGLE = false;

    /**
     * Set the speed of the motor. In other words, move it!
     * Don't forget to call this method with zero to stop the motor.
     */
    public void set(double value) {
        double calculated = rateLimiter.calculate(value * Constants.ArmConstants.kMaxArmOutput);

        // if (willRateLimit) {
        // if (getEncoderPositionDeg() > MAX_UP_DEG && calculated > 0) {
        // calculated = 0;
        // }

        // if (getEncoderPositionDeg() < MAX_DOWN_DEG && calculated < 0) {
        // calculated = 0;
        // }
        // }

        kRaiseTalonFX.set(calculated);

    }

    public static double degFromTicks(double ticks) {
        return ticks
                / BASE
                / PULSES_PER_REVOLUTION
                / GEARBOX_RATIO
                / SMALL_COG_TO_BIG_COG_1
                / BIG_COG_TO_BIG_COG_2
                * DEGREES_IN_A_CIRCLE
                + ANGLE_TO_ZERO;
    }

    public static double ticksFromDeg(double deg) {
        return (deg
                - ANGLE_TO_ZERO)
                / DEGREES_IN_A_CIRCLE
                * PULSES_PER_REVOLUTION
                * BIG_COG_TO_BIG_COG_2
                * SMALL_COG_TO_BIG_COG_1
                * GEARBOX_RATIO
                * BASE;
    }

    /**
     * Tried to create a command to bring the arm into its
     * usable range after resetting the encoder value.
     * 
     * // 6890
     * 
     * 
     * 
     * @deprecat NOT TESTED
     * 
     * @ a command for the {@link SmartDashboard}
     */
    // public Command raiseArmToZone() {
    // return Commands.runOnce(new Runnable() {
    // double encoderPositionDeg = ArmSubsystem.this.getEncoderPositionDeg();

    // /**
    // * Mock for {@link ArmSubsystem#getEncoderPositionDeg()}
    // *
    // * @return
    // */
    // private double getEncoderPositionDeg() {
    // return this.encoderPositionDeg;
    // }

    // /**
    // * {@link WPI_TalonFX#set(double)}
    // * Mock for {@link TalonFX#set(double)}
    // *
    // * @param value
    // */
    // private void kRaiseTalonFx$$set(double value) {
    // this.encoderPositionDeg += value;
    // }

    // private void toggleEncoderLimiting() {
    // SmartDashboard.putString("(Mock)end",
    // String.format("this.getEncoderPositionDeg() = %.2f, TARGET = %.2f",
    // this.getEncoderPositionDeg(), MAX_DOWN_DEG));

    // }

    // @Override
    // public void run() {
    // while (this.getEncoderPositionDeg() < MAX_DOWN_DEG) {
    // SmartDashboard.putNumber("(Mock)", this.getEncoderPositionDeg());

    // kRaiseTalonFx$$set(.2);
    // // ^^ kRaiseTalonFX.set(.2);

    // }

    // kRaiseTalonFx$$set(0);
    // // ^^ kRaiseTalonFX.set(0);

    // this.toggleEncoderLimiting();
    // }
    // });
    /*
     * () -> {
     * while (getEncoderPositionDeg() < MAX_DOWN_DEG) {
     * // kRaiseTalonFX.set(.2);
     * }
     * 
     * // kRaiseTalonFX.set(0);
     * toggleEncoderLimiting();
     * 
     * });
     */
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
    // public Command zeroTheCounter() {
    // return Commands.runOnce(() ->
    // this.kRaiseTalonFX.setSelectedSensorPosition(0));
    // }

    // public Command toggleEncoderLimiting() {
    // return Commands.runOnce(() -> willRateLimit = !willRateLimit);
    // }

    // public boolean isRateLimiting() {
    // return willRateLimit;
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", this.getEncoderPositionDeg());
        // SmartDashboard.putBoolean("Arm Limits", this.isRateLimiting());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));
    }
}
