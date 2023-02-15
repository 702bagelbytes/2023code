package frc.robot.subsystems;

import java.util.function.Supplier;
import java.lang.Math;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
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
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import static frc.robot.Constants.inchToMeter;

public class DriveSubsystem extends SubsystemBase {
    private final CANSparkMax sparkFL = new CANSparkMax(DriveConstants.SPARK_FL_ID, MotorType.kBrushed);
    private final RelativeEncoder leftEncoder = sparkFL.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    private final WPI_TalonSRX talonML = new WPI_TalonSRX(DriveConstants.TALON_ML_ID);
    private final WPI_TalonSRX talonBL = new WPI_TalonSRX(DriveConstants.TALON_BL_ID);
    private final MotorControllerGroup leftGroup = new MotorControllerGroup(sparkFL, talonML, talonBL);
    private final AHRSSubsystem ahrsSubsystem = new AHRSSubsystem();
    private Pose2d pose = new Pose2d();

    private final CANSparkMax sparkFR = new CANSparkMax(DriveConstants.SPARK_FR_ID, MotorType.kBrushed);
    private final RelativeEncoder rightEncoder = sparkFR.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    private final WPI_TalonSRX talonMR = new WPI_TalonSRX(DriveConstants.TALON_MR_ID);
    private final WPI_TalonSRX talonBR = new WPI_TalonSRX(DriveConstants.TALON_BR_ID);
    private final MotorControllerGroup rightGroup = new MotorControllerGroup(sparkFR, talonMR, talonBR);
    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            ahrsSubsystem.getRotation2d(), this.getLeftDistance(), this.getRightDistance());

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

    public double getRightDistance() {
        double wheelRadiusMeter = inchToMeter(DriveConstants.WHEEL_RADIUS_INCH);
        double circumference = 2 * wheelRadiusMeter * Math.PI;
        return rightEncoder.getPosition() * circumference;
    }

    public double getLeftDistance() {
        double wheelRadiusMeter = inchToMeter(DriveConstants.WHEEL_RADIUS_INCH);
        double circumference = 2 * wheelRadiusMeter * Math.PI;
        return leftEncoder.getPosition() * circumference;
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

    public Command arcadeDriveCmd(Supplier<Double> driveInput, Supplier<Double> rotationInput) {
        return this.runEnd(() -> drive.arcadeDrive(driveInput.get(), rotationInput.get()),
                () -> drive.arcadeDrive(0, 0));
    }

    /*
     * public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean
     * isFirstPath) {
     * return new SequentialCommandGroup(
     * new InstantCommand(() -> {
     * // Reset odometry for the first path you run during auto
     * if (isFirstPath) {
     * this.resetOdometry(traj.getInitialPose());
     * }
     * }),
     * new PPRamseteCommand(
     * traj,
     * this::getPose, // Pose supplier
     * new RamseteController(),
     * new SimpleMotorFeedforward(KS, KV, KA),
     * this.kinematics, // DifferentialDriveKinematics
     * this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
     * new PIDController(0, 0, 0), // Left controller. Tune these values for your
     * robot. Leaving them 0
     * // will only use feedforwards.
     * new PIDController(0, 0, 0), // Right controller (usually the same values as
     * left controller)
     * this::outputVolts, // Voltage biconsumer
     * true, // Should the path be automatically mirrored depending on alliance
     * color.
     * // Optional, defaults to true
     * this // Requires this drive subsystem
     * ));
     * }
     */
    private void resetOdometry(Pose2d initialPose) {

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Encoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.getPosition());
        var gyroAngle = ahrsSubsystem.getRotation2d();

        // Update the pose
        pose = m_odometry.update(gyroAngle,
                this.getLeftDistance(),
                this.getRightDistance());
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
