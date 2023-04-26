package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//feedforward

//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//if u want to current limit check this line out

//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX armTalonFX = new WPI_TalonFX(Constants.ArmConstants.ARM_TALON_ID);

    //if u figure this out then u don't need break mode which is good for conserving battery life.
    //slowly increase values from like 0.00001 to whatever 10 times it is 
    //and once it starts to move then u gradually increase until it is perfect.

    //use a binary search
    
    //private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);

    //figure out good values so arm doesn't draw too much power
   // SupplyCurrentLimitConfiguration config = new SupplyCurrentLimitConfiguration(false, 0, 0, 0);
    

    public ArmSubsystem() {
        //armTalonFX.configSupplyCurrentLimit()

        armTalonFX.configForwardSoftLimitThreshold(Constants.ArmConstants.MAX_UP_DEG);
        armTalonFX.configForwardSoftLimitEnable(Constants.ArmConstants.FORWARD_LIMIT_TOGGLE);
        armTalonFX.setNeutralMode(NeutralMode.Brake);

        

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

    public void set(double val) {
        
        armTalonFX.set(val * 0.4); //+ armFeedforward.calculate(Math.toRadians(getEncoderPositionDeg()), 0)* Constants.ArmConstants.kMaxArmOutput); // + Math.cos(Math.toRadians(getEncoderPositionDeg()) * 0.0001));

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", this.getEncoderPositionDeg());
    }

    public Command moveCmd(DoubleSupplier input) {
        return this.runEnd(() -> this.set(input.getAsDouble()), () -> this.set(0));

    }
}
