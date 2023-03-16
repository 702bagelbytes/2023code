package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AHRSSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class EncoderDriveCommand extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private AHRSSubsystem ahrsSubsystem;
    private PIDController leftController = new PIDController(0.35, 0, 0.005);
    private PIDController rightController = new PIDController(0.37, 0, 0.005);

    // temp for testing
    public static double kP = 0.5;
    public static double kI = 0;
    public static double kD = 0;
    public static double maxVel = 0;
    public static double maxAccel = 0;

    public EncoderDriveCommand(DriveSubsystem driveSubsystem, AHRSSubsystem ahrsSubsystem, double distance) {
        this.driveSubsystem = driveSubsystem;
        this.ahrsSubsystem = ahrsSubsystem;
        leftController.setSetpoint(distance);
        rightController.setSetpoint(distance);
        leftController.setTolerance(0.5);
        rightController.setTolerance(0.5);
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double leftSpeed = 0;
        if (!leftController.atSetpoint()) {
            leftSpeed = leftController.calculate(driveSubsystem.getLeftDistance());
        }
        double rightSpeed = 0;
        if (!rightController.atSetpoint()) {
            rightSpeed = rightController.calculate(driveSubsystem.getRightDistance());
        }
        driveSubsystem.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {

        boolean tilted = Math.abs(ahrsSubsystem.getBalanceAngle()) > 11;
        boolean atSetpoint = leftController.atSetpoint() && rightController.atSetpoint();
        return tilted || atSetpoint;
    }

    @Override
    public void end(boolean isInterrupted) {
        driveSubsystem.tankDrive(0, 0);
    }
}
