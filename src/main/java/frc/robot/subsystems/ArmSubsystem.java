package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private WPI_TalonSRX leftTalon = new WPI_TalonSRX(ArmConstants.ARM_LEFT_TALON);
    private CANSparkMax rightSpark = new CANSparkMax(ArmConstants.ARM_RIGHT_SPARK, MotorType.kBrushed);
    private RelativeEncoder encoder = rightSpark.getEncoder(
        SparkMaxRelativeEncoder.Type.kQuadrature, ArmConstants.ARM_COUNTS_PER_REV
    );

    public ArmSubsystem() {} 

    public RelativeEncoder getEncoder() {
        return encoder;
    }

    double getPower(double val) {
        if (val > 1) {
            // going down
            return ArmConstants.LOW_MULTIPLER;
        }
        if (encoder.getPosition() < -0.75) {
            // at the top and going up
            return ArmConstants.LOW_MULTIPLER;
        }
        return 1.0;
    }

    public void set(double val) {
        double correctedVal = -val;
        double power = getPower(val);
        leftTalon.set(correctedVal * ArmConstants.ARM_POWER * power);
        rightSpark.set(correctedVal * ArmConstants.ARM_POWER * power);
    }
}
