// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.subsystems.AHRSSubsystem;
// import frc.robot.subsystems.DriveSubsystem;

// public class AutoBalanceCommand extends CommandBase {
//     private final AHRSSubsystem ahrsSubsystem;
//     private final DriveSubsystem driveSubsystem;

//     public AutoBalanceCommand(DriveSubsystem driveSubsystem, AHRSSubsystem ahrsSubsystem) {
//         this.driveSubsystem = driveSubsystem;
//         this.ahrsSubsystem = ahrsSubsystem;

//         addRequirements(driveSubsystem);
//     }

//     public double getAngle() {
//         double angle = ahrsSubsystem.getBalanceAngle();
//         return angle;
//     }

//     Double initialAngle = null;

//     public double getInitialAngle() {
//         assert initialAngle != null : "please init first";
//         return this.initialAngle;
//     }

//     @Override
//     public void initialize() {
//         if (initialAngle == null) {
//             this.initialAngle = getAngle();
//         }
//     }

//     // @Override
//     // public void execute() {
//     //     driveSubsystem.tankDriveCmd(() -> -0.75, () -> -0.75).withTimeout(2)
//     //             .andThen(new WaitCommand(.2))
//     //             .andThen(new BalanceCommand(driveSubsystem, ahrsSubsystem::getBalanceAngle, getInitialAngle()));
//     // }

// }
