// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import java.lang.Math;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot.AutonomousChoices;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.EncoderDriveCommand;
import frc.robot.commands.TelescopePIDCommand;
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

                driveSubsystem.setDefaultCommand(driveSubsystem.tankDriveCmd(() -> -driverController.getLeftY(),
                                () -> -driverController.getRightY()));
                // driverController.a().whileTrue(new EncoderDriveCommand(driveSubsystem, 5));
                // driverController.b().onTrue(new
                // InstantCommand(driveSubsystem::resetEncoders));

                // Co-Driver
                armSubsystem.setDefaultCommand(armSubsystem.moveCmd(() -> -coDriverController.getLeftY()));
                coDriverController.povUp().whileTrue(telescopeSubsystem.moveCmd(() -> 1.0));
                coDriverController.povRight().whileTrue(telescopeSubsystem.moveCmd(() -> -1.0));
                coDriverController.b().onTrue(new ArmPIDCommand(armSubsystem, 0));
                coDriverController.x().onTrue(new TelescopePIDCommand(telescopeSubsystem, 0.05));
                coDriverController.povDown().onTrue(armSubsystem.resetEncodersCommand());
                coDriverController.povLeft().onTrue(telescopeSubsystem.resetEncodersCommand());
                coDriverController.rightTrigger(0.5).onTrue(grabotronSubsystem.toggleCommand());
                turretSubsystem.setDefaultCommand(turretSubsystem.runCmd(() -> coDriverController.getRightX()));
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
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 2.5)) // Score Mid
                        // .andThen(new TelescopePIDCommand(telescopeSubsystem, 5.6)) // Score High
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.1))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new ArmPIDCommand(armSubsystem, -65));

        private final Command BALANCE = new EncoderDriveCommand(driveSubsystem, ahrsSubsystem, -2.4)
                        .andThen(new BalanceCommand(driveSubsystem,
                                        ahrsSubsystem::getBalanceAngle,
                                        initialAngle));

        private final Command FULL = new ArmPIDCommand(armSubsystem, 16)
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 2)) // Score Mid
                        // .andThen(new TelescopePIDCommand(telescopeSubsystem, 5.6)) // Score High
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.1))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new ArmPIDCommand(armSubsystem, -65))
                        .andThen(new EncoderDriveCommand(driveSubsystem, ahrsSubsystem, -2.7)
                                        .andThen(new BalanceCommand(driveSubsystem,
                                                        ahrsSubsystem::getBalanceAngle,
                                                        initialAngle)));

        private final Command SCORE_MID_BACK_OUT = SCORE_MID
                        .andThen(new EncoderDriveCommand(driveSubsystem, ahrsSubsystem, -3));

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
                        case ArmScore:
                                return SCORE_MID;

                        case Balance:
                                return BALANCE;

                        case ScoreMidAndBackOut:
                                return SCORE_MID_BACK_OUT;

                        case Full:
                                return FULL;

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