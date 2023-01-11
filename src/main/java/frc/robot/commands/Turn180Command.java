package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurnConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Turn180Command extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final PIDController pid;

    public Turn180Command(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        this.pid = new PIDController(
            TurnConstants.kP, TurnConstants.kI, TurnConstants.kD
        );
        this.pid.setTolerance(TurnConstants.ANG_TOL, TurnConstants.VEL_TOL);
        this.pid.enableContinuousInput(-180, 180);
        // Don't addRequirements() on the driveSubsystem so that the driver can still
        //  control it.
    }

    public double normalizeAngle(double angle) {
        // avoid negative lhs in mod
        return ((angle + 180) % 360) - 180;
    }

    public double getAngle() {
        return driveSubsystem.gyroPidGet();
    }

    @Override
    public void initialize() {
        double angle = getAngle();
        double targetAngle = normalizeAngle(angle + 180);
        pid.setSetpoint(targetAngle);
        System.out.println("Turn180Command start");
    }

    @Override
    public void execute() {
        double angle = getAngle();
        double val = MathUtil.clamp(pid.calculate(angle), -1, 1); 
        driveSubsystem.setTurnAuto(val);
        System.out.println("Turn180Command execute");
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setTurnAuto(0);
        System.out.println("Turn180Command " + (interrupted ? "timed out" : "succeeded"));
    }
}
