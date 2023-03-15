package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.AHRSSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class GyroTurnToAngleCommand extends CommandBase {

    DriveSubsystem driveSubsystem; // drivetrain subsystem
    AHRSSubsystem ahrsSubsystem;
    double degreesToTurn; // the number of degrees we wish to turn
    double error; // How "incorrect" the current angle of the robot is as its moving
    double targetAngle; // targetAngle = initial angle + degreesToTurn

    /** Turns to an angle relative to the current angle using the gyro */
    public GyroTurnToAngleCommand(double degreesToTurn) {

        addRequirements(driveSubsystem);
        this.degreesToTurn = degreesToTurn;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.targetAngle = degreesToTurn + ahrsSubsystem.getYaw();
    }

    @Override
    public void execute() {
        error = targetAngle - ahrsSubsystem.getYaw();

        double speed = error * 0.07;

        if (Math.abs(speed) > 0.75) { // Maximum drive speed
            speed = Math.copySign(0.75, speed);
        }
        if (Math.abs(speed) < 0.15) { // Minimum drive speed
            speed = Math.copySign(0.15, speed);

        }

        driveSubsystem.tankDrive(speed, -speed); // drive with the calculated values
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.tankDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < 3; // End the command when we are within the specified threshold of our target
    }
}