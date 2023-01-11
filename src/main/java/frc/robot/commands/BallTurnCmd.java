package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BallTurnCmd extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final PIDController drivePID, turnPID;
    private double lastSeenTimestamp = -1;

    /**
     * this needs to be tweaked, as these numbers are designed for the competition robot ONLY
     */
    public BallTurnCmd(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        drivePID = new PIDController(0.07, 0, 0.02);
        drivePID.setSetpoint(43);
        turnPID = new PIDController(0.7, 0, 0.07);
        turnPID.setSetpoint(0);
    }

    @Override
    public void initialize() {
        System.out.println("BallTurnCmd started");
    }

    @Override
    public void execute() {
        double ballR = SmartDashboard.getNumber("BallR", -99);
        double ballX = SmartDashboard.getNumber("BallX", -99);
        if (10 <= ballR && ballR <= 40 && -1 <= ballX && ballX <= 1) {
            driveSubsystem.setDriveAuto(drivePID.calculate(ballR) * 0.5);
            driveSubsystem.setTurnAuto(-turnPID.calculate(ballX));
        } else {
            if (lastSeenTimestamp == -1) {
                lastSeenTimestamp = Timer.getFPGATimestamp();
            } else if (Timer.getFPGATimestamp() - lastSeenTimestamp >= 0.2) {
                driveSubsystem.setTurnAuto(0.5);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return SmartDashboard.getNumber("BallR", -99) >= 38;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setDriveAuto(0);
        driveSubsystem.setTurnAuto(0);
        if (interrupted) {
            System.out.println("BallTurnCmd interrupted");
        } else {
            System.out.println("BallTurnCmd finished");
        }
    }
}
