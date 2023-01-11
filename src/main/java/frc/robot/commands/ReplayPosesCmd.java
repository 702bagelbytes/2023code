package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Pose;

public class ReplayPosesCmd extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final boolean reverse;
    private int i;

    public ReplayPosesCmd(DriveSubsystem d, boolean reverse) {
        addRequirements(d);
        driveSubsystem = d;
        this.reverse = reverse;
    }

    @Override
    public void initialize() {
        i = reverse ? driveSubsystem.poses.size() - 1 : 0;
    }

    @Override
    public void execute() {
        Pose p = driveSubsystem.poses.get(i);
        System.out.println(
            "Execute pose " + i + "/" + driveSubsystem.poses.size() +
            ": " + p.drive + " " + p.strafe + " " + p.turn
        );
        double multiplier = reverse ? -1 : 1;
        driveSubsystem.driveCartesian(
            p.drive * multiplier, p.strafe * multiplier, p.turn * multiplier, false
        );
        i += multiplier;
    }

    @Override
    public boolean isFinished() {
        // Don't run if logging is enabled!
        return (
            driveSubsystem.loggingEnabled ||
            i < 0 ||
            i >= driveSubsystem.poses.size()
        );
    }
}
