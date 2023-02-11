package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax sparkFL = new CANSparkMax(DriveConstants.SPARK_FL_ID, MotorType.kBrushed);
    private final RelativeEncoder leftEncoder = sparkFL.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    private final WPI_TalonSRX talonML = new WPI_TalonSRX(DriveConstants.TALON_ML_ID);
    private final WPI_TalonSRX talonBL = new WPI_TalonSRX(DriveConstants.TALON_BL_ID);
    private final MotorControllerGroup leftGroup = new MotorControllerGroup(sparkFL, talonML, talonBL);

    private final CANSparkMax sparkFR = new CANSparkMax(DriveConstants.SPARK_FR_ID, MotorType.kBrushed);
    private final RelativeEncoder rightEncoder = sparkFR.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    private final WPI_TalonSRX talonMR = new WPI_TalonSRX(DriveConstants.TALON_MR_ID);
    private final WPI_TalonSRX talonBR = new WPI_TalonSRX(DriveConstants.TALON_BR_ID);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(sparkFR, talonMR, talonBR);

    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    public DriveSubsystem() {
        sparkFL.setInverted(true);
        sparkFR.setInverted(true);
        leftGroup.setInverted(true);
        rightGroup.setInverted(false);

        sparkFL.setIdleMode(IdleMode.kBrake);
        talonML.setNeutralMode(NeutralMode.Brake);
        talonBL.setNeutralMode(NeutralMode.Brake);
        sparkFR.setIdleMode(IdleMode.kBrake);
        talonMR.setNeutralMode(NeutralMode.Brake);
        talonBR.setNeutralMode(NeutralMode.Brake);

        leftEncoder.setInverted(DriveConstants.LEFT_ENCODER_INVERTED);
        rightEncoder.setInverted(DriveConstants.RIGHT_ENCODER_INVERTED);

        drive.setMaxOutput(DriveConstants.DRIVE_SPEED);
    }

    public double clampSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        if (Math.abs(speed) <= 0.05) {
            return 0;
        }
        double minSpeed = 0.2;
        return Math.copySign(minSpeed, speed) + (1 - minSpeed) * speed;
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        SmartDashboard.putNumber("Raw Left Speed", leftSpeed);
        SmartDashboard.putNumber("Raw Right Speed", rightSpeed);
        leftSpeed = clampSpeed(leftSpeed);
        rightSpeed = clampSpeed(rightSpeed);
        SmartDashboard.putNumber("Left Speed", leftSpeed);
        SmartDashboard.putNumber("Right Speed", rightSpeed);
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public Command tankDriveCmd(Supplier<Double> leftSpeedSupplier, Supplier<Double> rightSpeedSupplier) {
        return this.runEnd(
                () -> tankDrive(leftSpeedSupplier.get(),
                        rightSpeedSupplier.get()),
                () -> drive.tankDrive(0, 0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
    }

    public double getLeftEncoderValue() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderValue() {
        return rightEncoder.getPosition();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
}
