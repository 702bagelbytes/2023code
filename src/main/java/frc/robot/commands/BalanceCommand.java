package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Float> balanceAngleSupplier;
    private static final PIDController controller = new PIDController(0.05, 0.0, 0.005);

    public BalanceCommand(DriveSubsystem driveSubsystem,
            Supplier<Float> balanceAngleSupplier, double setpoint) {
        this.driveSubsystem = driveSubsystem;
        this.balanceAngleSupplier = balanceAngleSupplier;
        controller.setTolerance(1);
        controller.setSetpoint(setpoint);
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {

        double leftSpeed = controller.calculate(balanceAngleSupplier.get());
        double rightSpeed = controller.calculate(balanceAngleSupplier.get());
        leftSpeed = MathUtil.clamp(leftSpeed, 0.05, 0.64);
        rightSpeed = MathUtil.clamp(rightSpeed, 0.05, 0.72);
        SmartDashboard.putNumber("L_AutoSpeed", leftSpeed);
        SmartDashboard.putNumber("R_AutoSpeed", rightSpeed);
        driveSubsystem.tankDrive(-leftSpeed, -rightSpeed);
    }

    @Override

    public boolean isFinished() {
        boolean atSetpoint = controller.atSetpoint();
        return atSetpoint;

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Auto Speed", 0);
        driveSubsystem.tankDrive(0, 0);
    }
}
