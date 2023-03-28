// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot.AutonomousChoices;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.EncoderDriveCommand;
import frc.robot.commands.MoveBackwardsCommand;
import frc.robot.commands.MoveForwardCommand;
import frc.robot.commands.TelescopePIDCommand;
import frc.robot.commands.TurretPIDCommand;
import frc.robot.subsystems.AHRSSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GRABOTRONSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArmPIDCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        private final AHRSSubsystem ahrsSubsystem = new AHRSSubsystem();
        private final DriveSubsystem driveSubsystem = new DriveSubsystem(() -> ahrsSubsystem.getRotation2d());
        private final TurretSubsystem turretSubsystem = new TurretSubsystem();
        private final GRABOTRONSubsystem grabotronSubsystem = new GRABOTRONSubsystem();
        private final TelescopeSubsystem telescopeSubsystem = new TelescopeSubsystem();
        private final ArmSubsystem armSubsystem = new ArmSubsystem();
        private final CommandXboxController driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);
        private final CommandXboxController coDriverController = new CommandXboxController(
                        OperatorConstants.kCoDriverControllerPort);
        float initialAngle;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureButtonBindings();

                // SmartDashboard.putData("Zero Arm Angle", (Sendable)
                // armSubsystem.zeroTheCounter());
                // SmartDashboard.putData("Toggle Arm Limits", (Sendable)
                // armSubsystem.toggleEncoderLimiting());
                // SmartDashboard.putData("Up To Zone", (Sendable)
                // armSubsystem.raiseArmToZone());
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // Driver

                driveSubsystem.setDefaultCommand(driveSubsystem.tankDriveCmd(
                                () -> {
                                        if (driverController.rightTrigger().getAsBoolean()) {
                                                return 0.5 * -driverController.getLeftY();
                                        } else {
                                                return -driverController.getLeftY();
                                        }
                                },
                                () -> {
                                        if (driverController.rightTrigger().getAsBoolean()) {
                                                return 0.5 * -driverController.getRightY();
                                        } else {
                                                return -driverController.getRightY();
                                        }
                                }));
                // driverController.a().whileTrue(new EncoderDriveCommand(driveSubsystem, 5));
                // driverController.b().onTrue(new
                // InstantCommand(driveSubsystem::resetEncoders));

                // Co-Driver
                // armSubsystem.moveCmd(() -> -coDriverController.getLeftY());
                // armSubsystem.setDefaultCommand(armSubsystem.moveCmd(() ->
                // -coDriverController.getLeftY()));
                // coDriverController.y().whileTrue(telescopeSubsystem.moveCmd(() -> 1.0));
                // coDriverController.a().whileTrue(telescopeSubsystem.moveCmd(() -> -1.0));
                // coDriverController.x().onTrue(new TelescopePIDCommand(telescopeSubsystem,
                // 0.02));

                // coDriverController.leftBumper().onTrue(new ArmPIDCommand(armSubsystem, 0));
                // coDriverController.povDown().onTrue(armSubsystem.resetEncodersCommand());
                // coDriverController.povLeft().onTrue(telescopeSubsystem.resetEncodersCommand());
                // coDriverController.rightTrigger(0.5).onTrue(grabotronSubsystem.toggleCommand());
                // turretSubsystem.setDefaultCommand(turretSubsystem.runCmd(() ->
                // coDriverController.getRightX()));
        }

        public void resetGyro() {
                this.ahrsSubsystem.resetGyro();
        }

        public void calibrateGyro() {
                this.ahrsSubsystem.calibrateGyro();
        }

        public void resetDriveEncoders() {
                this.driveSubsystem.resetEncoders();
        }

        public void setArmBrakeMode(NeutralMode newMode) {
                this.armSubsystem.setBrakeMode(newMode);
        }

        private final Command SCORE_MID = new ArmPIDCommand(armSubsystem, 16)
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 3.2)) // Score Mid
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.05))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new ArmPIDCommand(armSubsystem, -69))
                        .andThen(new MoveBackwardsCommand(driveSubsystem).withTimeout(2.5));

        private final Command bumpAlone = new MoveForwardCommand(driveSubsystem).withTimeout(0.5);
        PathPlannerTrajectory twoPieceTest = PathPlanner.loadPath("TwoGamePieceAutoL", 1, 0.5);
        PathPlannerTrajectory twoPieceTest2 = PathPlanner.loadPath("TwoGamePieceAuto2", 1, 0.5);
        private final Command TWO_PIECE_AUTO = new ArmPIDCommand(armSubsystem, 16)
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 3.2)) // Score Mid
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.05))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new ArmPIDCommand(armSubsystem, -69))
                        .andThen(driveSubsystem.followTrajectoryCommand(twoPieceTest, true, false))
                        .andThen(new ArmPIDCommand(armSubsystem, -50))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 1))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.05))
                        .andThen(driveSubsystem.followTrajectoryCommand(twoPieceTest2, false, false))
                        .andThen(new ArmPIDCommand(armSubsystem, 16))
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 3.2)) // Score Mid
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(Commands.parallel(new TelescopePIDCommand(telescopeSubsystem, 0.05),
                                        grabotronSubsystem.toggleCommand()))
                        .andThen(new ArmPIDCommand(armSubsystem, -69));

        private final Command BALANCE = new MoveForwardCommand(driveSubsystem).withTimeout(0.5).andThen(

                        new MoveBackwardsCommand(driveSubsystem).withTimeout(3.3)
                                        // .andThen(new MoveForwardCommand(driveSubsystem).withTimeout(2));
                                        .andThen(new BalanceCommand(driveSubsystem, ahrsSubsystem::getBalanceAngle,
                                                        0)));

        private final Command SCORE_LOW = new ArmPIDCommand(armSubsystem, -50)
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 1))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.05))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new ArmPIDCommand(armSubsystem, -69))
                        .andThen(new MoveBackwardsCommand(driveSubsystem).withTimeout(2.5));

        PathPlannerTrajectory ScoreHigh = PathPlanner.loadPath("ScoreHigh", 1, 0.5);
        PathPlannerTrajectory ScoreHigh2 = PathPlanner.loadPath("ScoreHigh2", 1, 0.5);

        private final Command TWO_PIECE_HIGH_AUTO = new ArmPIDCommand(armSubsystem, -55)
                        .andThen(new TurretPIDCommand(turretSubsystem, 90))
                        .andThen(new ArmPIDCommand(armSubsystem, 19))
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 5.6))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.05))
                        .andThen(new ArmPIDCommand(armSubsystem, -55))
                        .andThen(new TurretPIDCommand(turretSubsystem, 0))
                        .andThen(driveSubsystem.followTrajectoryCommand(ScoreHigh, true, false))
                        .andThen(new ArmPIDCommand(armSubsystem, -50))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 1))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.05))
                        .andThen(driveSubsystem.followTrajectoryCommand(ScoreHigh2, false, false))
                        .andThen(new ArmPIDCommand(armSubsystem, -55))
                        .andThen(new TurretPIDCommand(turretSubsystem, 90))
                        .andThen(new ArmPIDCommand(armSubsystem, 19))
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 5.6))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.05))
                        .andThen(new ArmPIDCommand(armSubsystem, -55))
                        .andThen(new TurretPIDCommand(turretSubsystem, 0));

        private final Command BumpBackOut = new MoveForwardCommand(driveSubsystem).withTimeout(0.5)
                        .andThen(new MoveBackwardsCommand(driveSubsystem).withTimeout(2));

        private final Command SCORE_MID_BACK_OUT = SCORE_MID;

        PathPlannerTrajectory NoArmlol = PathPlanner.loadPath("HeeHeeNoArmAuto", 1, 0.5);
        PathPlannerTrajectory NoArmlol2 = PathPlanner.loadPath("HeeHeeNoArmAuto2", 1, 0.5);

        private final Command BUMP_SCORE_SECOND_PIECE = new MoveForwardCommand(driveSubsystem).withTimeout(0.5)
                        .andThen(driveSubsystem.followTrajectoryCommand(NoArmlol, true, true))
                        .andThen(driveSubsystem.followTrajectoryCommand(NoArmlol2, true, false));

        private final Command DEFAULT = Commands.runOnce(() -> {
                System.out.println(":)");

        });

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand(AutonomousChoices autoCode) {
                initialAngle = ahrsSubsystem.getBalanceAngle();
                SmartDashboard.putNumber("Balance angle ", initialAngle);
                switch (autoCode) {
                        case BumpAlone:
                                return bumpAlone;

                        case TwoPieceAuto:
                                return TWO_PIECE_AUTO;
                        case ScoreHighTwice:
                                return TWO_PIECE_HIGH_AUTO;

                        case ArmScore:
                                return SCORE_MID;

                        case Balance:
                                return BALANCE;
                        case BumpBackOut:
                                return BumpBackOut;
                        case BumpScoreSecondPiece:
                                return BUMP_SCORE_SECOND_PIECE;

                        case ScoreMidAndBackOut:
                                return SCORE_MID_BACK_OUT;

                        case ScoreLowAndBackOut:
                                return SCORE_LOW;

                        case PathTest:
                                PathPlannerTrajectory examplePath = PathPlanner.loadPath("DriveStraight",
                                                new PathConstraints(1, .5));

                                return driveSubsystem.followTrajectoryCommand(examplePath, true, false);
                        case PathTestBackwards:
                                PathPlannerTrajectory examplePath2 = PathPlanner.loadPath("DriveBackwards",
                                                new PathConstraints(1, .5), true);

                                return driveSubsystem.followTrajectoryCommand(examplePath2, true, true);
                        default:
                                return DEFAULT;
                }
        }

}