package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX armTalonFX = new WPI_TalonFX(Constants.ArmConstants.ARM_TALON_ID);
    private final SlewRateLimiter rateLimiter = new SlewRateLimiter(4.0);

    public ArmSubsystem() {
        armTalonFX.setNeutralMode(NeutralMode.Brake);
        armTalonFX.configForwardSoftLimitThreshold(Constants.ArmConstants.MAX_UP_DEG);
        armTalonFX.configForwardSoftLimitEnable(Constants.ArmConstants.FORWARD_LIMIT_TOGGLE);

    }

    public void resetEncoders() {
        armTalonFX.setSelectedSensorPosition(0);
    }

    public Command resetEncodersCommand() {

        return this.runOnce(() -> resetEncoders());
    }

    public double getEncoderPositionDeg() {

        return degFromTicks(armTalonFX.getSelectedSensorPosition());
    }

    // private boolean willRateLimit = false;

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

        armTalonFX.set(calculated);

    }

    public static double degFromTicks(double ticks) {
        return ticks
                / Constants.EncoderConstants.BASE
                / Constants.EncoderConstants.PULSES_PER_REVOLUTION
                / Constants.EncoderConstants.GEARBOX_RATIO
                / Constants.EncoderConstants.SMALL_COG_TO_BIG_COG_1
                / Constants.EncoderConstants.BIG_COG_TO_BIG_COG_2
                * Constants.EncoderConstants.DEGREES_IN_A_CIRCLE
                + Constants.EncoderConstants.ANGLE_TO_ZERO;
    }

    public static double ticksFromDeg(double deg) {
        return (deg
                - Constants.EncoderConstants.ANGLE_TO_ZERO)
                / Constants.EncoderConstants.DEGREES_IN_A_CIRCLE
                * Constants.EncoderConstants.PULSES_PER_REVOLUTION
                * Constants.EncoderConstants.BIG_COG_TO_BIG_COG_2
                * Constants.EncoderConstants.SMALL_COG_TO_BIG_COG_1
                * Constants.EncoderConstants.GEARBOX_RATIO
                * Constants.EncoderConstants.BASE;
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
