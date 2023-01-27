package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EncoderDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private double encoderRots; // only straight driving is supported for now
    private ProfiledPIDController leftController;
    private ProfiledPIDController rightController;

    // temp for testing
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double maxVel = 0;
    public static double maxAccel = 0;

    public static void updateConstants() {
        Function<Double, Double> c = (n) -> (n + Math.random() * 0.000001);
        kP = SmartDashboard.getNumber("kP", 0.1);
        SmartDashboard.putNumber("kP", c.apply(kP));
        kI = SmartDashboard.getNumber("kI", 0);
        SmartDashboard.putNumber("kI", c.apply(kI));
        kD = SmartDashboard.getNumber("kD", 0);
        SmartDashboard.putNumber("kD", c.apply(kD));
        maxVel = SmartDashboard.getNumber("maxVel", 7);
        SmartDashboard.putNumber("maxVel", c.apply(maxVel));
        maxAccel = SmartDashboard.getNumber("maxAccel", 15);
        SmartDashboard.putNumber("maxAccel", c.apply(maxAccel));
    }

    static ProfiledPIDController createController() {
        return new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVel, maxAccel));
    }

    public EncoderDriveCommand(DriveSubsystem driveSubsystem, double encoderRots) {
        this.driveSubsystem = driveSubsystem;
        this.encoderRots = encoderRots;
        this.leftController = null;
        this.rightController = null;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        updateConstants();
        this.leftController = createController();
        this.rightController = createController();
        this.leftController.setGoal(driveSubsystem.getLeftEncoderValue() + encoderRots);
        this.rightController.setGoal(driveSubsystem.getRightEncoderValue() + encoderRots);
    }

    @Override
    public void execute() {
        double leftSpeed = 0;
        if (!leftController.atGoal()) {
            leftSpeed = leftController.calculate(driveSubsystem.getLeftEncoderValue());
        }
        double rightSpeed = 0;
        if (!rightController.atGoal()) {
            rightSpeed = rightController.calculate(driveSubsystem.getRightEncoderValue());
        }
        SmartDashboard.putNumber("Left Goal", leftController.getGoal().position);
        SmartDashboard.putNumber("Right Goal", rightController.getGoal().position);
        driveSubsystem.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {
        return leftController.atGoal() && rightController.atGoal();
    }

    @Override
    public void end(boolean isInterrupted) {
        driveSubsystem.tankDrive(0, 0);
    }
}
