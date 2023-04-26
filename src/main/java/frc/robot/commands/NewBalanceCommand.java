package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
public class NewBalanceCommand extends CommandBase{
    DriveSubsystem driveSubsystem;
    private final Supplier<Float> balanceAngleSupplier;
    //worked twice in a row, might want to try to make the PID version of this work in case we like fall off the ramp or something

    public NewBalanceCommand(DriveSubsystem driveSubsystem,  Supplier<Float> balanceAngleSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.balanceAngleSupplier = balanceAngleSupplier;
        addRequirements(driveSubsystem);

    }
    @Override
    public void execute() {
        driveSubsystem.tankDrive(-0.64, -0.72); //.65 .72
     
    }

    @Override

    public boolean isFinished() {
        if(Math.abs(balanceAngleSupplier.get()) < 10) {
            return true;
        }
        else {
            return false;
        }
        

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Auto Speed", 0);
        driveSubsystem.tankDrive(0, 0);
    }
}




    

