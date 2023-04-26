package frc.robot.subsystems;

import java.util.function.Supplier;
import java.lang.Math;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants.MotorFeed;
// import frc.robot.Constants.*;
import frc.robot.Constants.DriveConstants.Ramsete;

import static frc.robot.Constants.inchToMeter;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax sparkFL = new CANSparkMax(DriveConstants.SPARK_FL_ID, MotorType.kBrushed);
    private final RelativeEncoder leftEncoder = sparkFL.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    private final WPI_TalonSRX talonML = new WPI_TalonSRX(DriveConstants.TALON_ML_ID);
    private final WPI_TalonSRX talonBL = new WPI_TalonSRX(DriveConstants.TALON_BL_ID);
    private final MotorControllerGroup leftGroup = new MotorControllerGroup(sparkFL, talonML, talonBL);
    DifferentialDriveOdometry m_odometry;
    private final Supplier<Rotation2d> getRotation2d;

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20));

    private Pose2d pose = new Pose2d();

    private final CANSparkMax sparkFR = new CANSparkMax(DriveConstants.SPARK_FR_ID, MotorType.kBrushed);
    private final RelativeEncoder rightEncoder = sparkFR.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    private final WPI_TalonSRX talonMR = new WPI_TalonSRX(DriveConstants.TALON_MR_ID);
    private final WPI_TalonSRX talonBR = new WPI_TalonSRX(DriveConstants.TALON_BR_ID);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(sparkFR, talonMR, talonBR);
    private final DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

    public DriveSubsystem(Supplier<Rotation2d> getRotation2d) {
        //this is probably wrong I would scrap and redo for swerve 100%
        this.getRotation2d = getRotation2d;
        this.m_odometry = new DifferentialDriveOdometry(
                getRotation2d.get(), this.getLeftDistance(), this.getRightDistance(), pose);

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

        //set current limits that the tech guy at LA Regional 2023 told me to do, maybe ask Reza 
        //if current limiting the arm or telescoping would be worth it

        sparkFL.setSmartCurrentLimit(30);
        sparkFR.setSmartCurrentLimit(30);
        talonML.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 0.1));
        talonMR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 0.1));
        talonBL.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 0.1));
        talonBR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 40, 0.1));
        leftEncoder.setInverted(DriveConstants.LEFT_ENCODER_INVERTED);
        rightEncoder.setInverted(DriveConstants.RIGHT_ENCODER_INVERTED);

        resetEncoders();

        //haha so yeah Soleil, when I said we were at full drive speed, this next comment wasn't commented out
        //, which means that we were definitely not at full speed xD lol :) :((((

        //drive.setMaxOutput(DriveConstants.DRIVE_SPEED);
    }

    public double getRightDistance() {
        double wheelRadiusMeter = inchToMeter(DriveConstants.WHEEL_RADIUS_INCH);
        double circumference = 2 * wheelRadiusMeter * Math.PI;
        return (rightEncoder.getPosition() - DriveConstants.ENCODER_OFFSET) * circumference;
    }

    public double getLeftDistance() {
        double wheelRadiusMeter = inchToMeter(DriveConstants.WHEEL_RADIUS_INCH);
        double circumference = 2 * wheelRadiusMeter * Math.PI;
        return (leftEncoder.getPosition() - DriveConstants.ENCODER_OFFSET) * circumference;
    }

    public double clampSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        if (Math.abs(speed) <= 0.05) {
            return 0;
        }
        return speed;
    }

    public double chargeStationClampSpeed(double speed) {
        speed = MathUtil.clamp(speed, -0.5, 0.5);
        if (Math.abs(speed) <= 0.05) {
            return 0;
        }
        double minSpeed = 0.2;
        return Math.copySign(minSpeed, speed) + (1 - minSpeed) * speed;
    }

    public void arcadeDrive(double speed, double rotation) {
        speed = clampSpeed(speed);
        SmartDashboard.putNumber("Speed", speed);
        drive.arcadeDrive(speed, rotation, true);

    }

    public Command arcadeDriveCmd(Supplier<Double> speedSupplier, Supplier<Double> rotationSupplier) {
        return this.runEnd(
                () -> arcadeDrive(clampSpeed(speedSupplier.get()), rotationSupplier.get() *.85),
                () -> drive.arcadeDrive(0, 0));
    }

     public void tankDrive(double leftSpeed, double rightSpeed) {
        // leftSpeed = leftLimiter.calculate(clampSpeed(leftSpeed));
        leftSpeed = clampSpeed(leftSpeed);
        // rightSpeed = rightLimiter.calculate(clampSpeed(rightSpeed));
        rightSpeed = clampSpeed(rightSpeed);
        SmartDashboard.putString("Speed", String.format("L: %.2f, R: %.2f}", leftSpeed, rightSpeed));
        drive.tankDrive(leftSpeed, rightSpeed);
    }

    public Command tankDriveCmd(Supplier<Double> leftSpeedSupplier, Supplier<Double> rightSpeedSupplier) {
        return this.runEnd(
                () -> tankDrive(leftSpeedSupplier.get(),
                        rightSpeedSupplier.get()),
                () -> drive.tankDrive(0, 0));
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
        //return pose;  is this correct?
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {

        return new DifferentialDriveWheelSpeeds(
                leftEncoder.getVelocity() * (2 * Math.PI * Units.inchesToMeters(DriveConstants.WHEEL_RADIUS_INCH) / 60),
                rightEncoder.getVelocity()
                        * (2 * Math.PI * Units.inchesToMeters(DriveConstants.WHEEL_RADIUS_INCH) / 60));

    }

    public void outputVoltsNegative(double left, double right) {
        SmartDashboard.putNumber("left volts", -left);
        SmartDashboard.putNumber("right volts", right);
        this.leftGroup.setVoltage(-left);
        this.rightGroup.setVoltage(right);
    }

    public void outputVolts(double left, double right) {
        SmartDashboard.putNumber("left volts", left);
        SmartDashboard.putNumber("right volts", right);
        this.leftGroup.setVoltage(left);
        this.rightGroup.setVoltage(right);
    }

    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath,
            boolean negativeOutputVolts) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialPose());
                    }
                }),
                new PPRamseteCommand(
                        traj,
                        this::getPose,
                        new RamseteController(),
                        new SimpleMotorFeedforward(MotorFeed.KS, MotorFeed.KV, MotorFeed.KA),
                        this.kinematics,
                        this::getWheelSpeeds,
                        new PIDController(Ramsete.P, Ramsete.I, Ramsete.D),
                        new PIDController(Ramsete.P, Ramsete.I, Ramsete.D),
                        negativeOutputVolts ? this::outputVoltsNegative : this::outputVolts,
                        this));
    }

    private void resetOdometry(Pose2d initialPose) {
        m_odometry.resetPosition(getRotation2d.get(), this.getLeftDistance(), this.getRightDistance(), initialPose);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Velocity", leftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Velocity", rightEncoder.getVelocity());
        
        SmartDashboard.putString("Drive Encoders",
                String.format("L: %.2f, R: %.2f", leftEncoder.getPosition(), rightEncoder.getPosition()));
        var gyroAngle = getRotation2d.get();

        // Update the pose
        pose = m_odometry.update(gyroAngle,
                this.getLeftDistance(),
                this.getRightDistance());
        SmartDashboard.putNumber("Left distance", getLeftDistance());
        SmartDashboard.putNumber("Right distance", getLeftDistance());
    }

    public double getLeftEncoderValue() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderValue() {
        return rightEncoder.getPosition();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(DriveConstants.ENCODER_OFFSET);
        rightEncoder.setPosition(DriveConstants.ENCODER_OFFSET);
    }
}
