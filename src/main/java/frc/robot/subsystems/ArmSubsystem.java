package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX armTalonFX = new WPI_TalonFX(Constants.ArmConstants.ARM_TALON_ID);

    public ArmSubsystem() {
        armTalonFX.configForwardSoftLimitThreshold(Constants.ArmConstants.MAX_UP_DEG);
        armTalonFX.configForwardSoftLimitEnable(Constants.ArmConstants.FORWARD_LIMIT_TOGGLE);

    }

    public void setBrakeMode(NeutralMode newMode) {
        armTalonFX.setNeutralMode(newMode);
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

    /**
     * Set the speed of the motor. In other words, move it!
     * Don't forget to call this method with zero to stop the motor.
     */
    public void set(double value) {
        double calculated = value * Constants.ArmConstants.kMaxArmOutput;

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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", this.getEncoderPositionDeg());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));
    }
}
