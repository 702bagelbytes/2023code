package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX armTalonFX = new WPI_TalonFX(Constants.ArmConstants.ARM_TALON_ID);
    private final PIDController armPIDController = new PIDController(0.08, 0, 0.01);
    private double setpoint;
    RobotContainer robotContainer;

    public ArmSubsystem() {
        armTalonFX.configForwardSoftLimitThreshold(Constants.ArmConstants.MAX_UP_DEG);
        armTalonFX.configForwardSoftLimitEnable(Constants.ArmConstants.FORWARD_LIMIT_TOGGLE);
        armTalonFX.setNeutralMode(NeutralMode.Brake);
        armPIDController.setTolerance(4);
        setpoint = getEncoderPositionDeg();

    }

    public WPI_TalonFX getTalon() {
        return armTalonFX;
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

    public PIDController getArmPID() {
        return armPIDController;
    }

    public void set() {
        double calculated = armPIDController.calculate(getEncoderPositionDeg(), setpoint)
                * Constants.ArmConstants.kMaxArmOutput;
        armTalonFX.set(calculated + robotContainer.getCoDriverController().getLeftY() * 0.2);

    }

    public void setOld(double val) {
        double speed = val * Constants.ArmConstants.kMaxArmOutput;
        armTalonFX.set(speed);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", this.getEncoderPositionDeg());
        // if (setpoint <= -73) {
        // setpoint = -73;
        // }

        // else if (setpoint >= 20) {
        // setpoint = 20;
        // }
        // this.set();
    }

    public Command setCmd(double newSetpoint) {

        return this.runOnce(() -> setpoint = newSetpoint);
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runOnce(() -> setpoint = (input.getAsDouble() * 45) - 25);
        // this.runEnd(() -> this.setOld(input.getAsDouble()), () -> this.setOld(0));

    }
}
