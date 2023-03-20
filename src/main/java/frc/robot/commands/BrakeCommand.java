
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BrakeCommand extends CommandBase {
    PIDController controller = new PIDController(0.05, 0, 0);
    double setPoint = 0;
    DriveSubsystem driveSubsystem;

    public BrakeCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Brake P", 0.01);
    }

    @Override
    public void execute() {
        controller.setP(SmartDashboard.getNumber("Brake P", 0.01));

    }

}