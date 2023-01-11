package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

public class LoggerCmd extends CommandBase{
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private RelativeEncoder armEncoder;

    public LoggerCmd(ClimberSubsystem climber, ArmSubsystem arm) {
        this.leftEncoder = climber.getLeftEncoder();
        this.rightEncoder = climber.getRightEncoder();
        this.armEncoder = arm.getEncoder();
        // no need to addRequirements()
    } 

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void initialize() {
        System.out.println("LoggerCmd initialize");
        resetEncoders();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("ClimberL", leftEncoder.getDistance());
        SmartDashboard.putNumber("ClimberR", rightEncoder.getDistance());
        SmartDashboard.putNumber("Arm", armEncoder.getPosition());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
        armEncoder.setPosition(0);
    }
}
