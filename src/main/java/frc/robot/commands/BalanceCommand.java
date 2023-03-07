package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final Supplier<Double> balanceAngleSupplier;
    // private static final PIDController controller = new PIDController(0.05, 0.0, 0.005);

    public BalanceCommand(DriveSubsystem driveSubsystem,
            Supplier<Double> balanceAngleSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.balanceAngleSupplier = balanceAngleSupplier;
        // controller.setTolerance(2);
        // controller.setSetpoint(setpoint);
        addRequirements(driveSubsystem);


    }

    double balanceAngle() {
        return balanceAngleSupplier.get();
    }

    @Override
    public void execute() {
        final double balanceAngle = balanceAngle();
        final double ZERO_ANGLE = 1.1;

        if (fitsWithError(balanceAngle, ZERO_ANGLE, 2)) {
            driveSubsystem.tankDrive(0, 0);
            return;
        }

        SmartDashboard.putString("Balancing Status", "Balancing...");
        SmartDashboard.putString("BB Angle", String.format("orig: %.3f, modded: %.3f", balanceAngleSupplier.get(), balanceAngle()));   

        final double SPEED = .4;

        if (balanceAngle < 0) {
            driveSubsystem.tankDrive(SPEED, SPEED);
        }

        if (balanceAngle > 0) {
            driveSubsystem.tankDrive(-SPEED, -SPEED);
        }

        // double speed = driveSubsystem.chargeStationClampSpeed(controller.calculate(balanceAngleSupplier.get()));
        // SmartDashboard.putData(controller);
        // SmartDashboard.putNumber("Auto Speed", speed);
        // driveSubsystem.tankDrive(speed, speed);
    }

    /**
     * Range utility
     * @param value an input value
     * @param expected the expected value with which to match `value`.
     * @param error the allotted error value
     * @return whether `value` fits in the range: (expected-error) < value < (expected+error)
     */
    public static boolean fitsWithError(double value, double expected, double error) {
        double topBound = expected + error;
        double bottomBound = expected - error;

        return value >= bottomBound && value <= topBound;
    }

    @Override
    public boolean isFinished() {
        return false;
        // return fitsWithError(this.balanceAngle(), 0, 1);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Auto Speed", 0);
        SmartDashboard.putString("Balancing Status", "Balanced!!!");

        driveSubsystem.tankDrive(0, 0);
    }
}
