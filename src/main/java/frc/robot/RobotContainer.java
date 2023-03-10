// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import java.lang.Math;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
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
                // driveSubsystem.setDefaultCommand(
                // driveSubsystem.tankDriveCmd(() -> -driverController.getLeftY(), () ->
                // -driverController.getRightY()));
                driveSubsystem.setDefaultCommand(driveSubsystem.tankDriveCmd(() -> -driverController.getLeftY(),
                                () -> -driverController.getRightY()));
                // driverController.a().whileTrue(new EncoderDriveCommand(driveSubsystem, 5));
                // driverController.b().onTrue(new
                // InstantCommand(driveSubsystem::resetEncoders));

                // Co-Driver
                armSubsystem.setDefaultCommand(armSubsystem.moveCmd(() -> -coDriverController.getLeftY()));
                coDriverController.y().whileTrue(telescopeSubsystem.moveCmd(() -> 1.0));
                coDriverController.a().whileTrue(telescopeSubsystem.moveCmd(() -> -1.0));
                coDriverController.povDown().onTrue(armSubsystem.resetEncodersCommand());
                coDriverController.povLeft().onTrue(telescopeSubsystem.resetEncodersCommand());
                coDriverController.rightTrigger(0.5).onTrue(grabotronSubsystem.toggleCommand());
                turretSubsystem.setDefaultCommand(turretSubsystem.runCmd(() -> coDriverController.getRightX()));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                return armSubsystem.resetEncodersCommand()
                                .andThen(telescopeSubsystem.resetEncodersCommand())
                                .andThen(new ArmPIDCommand(armSubsystem, 16))
                                .andThen(new ArmPIDCommand(armSubsystem, 10))
                                .andThen(new TelescopePIDCommand(telescopeSubsystem, 5.6))
                                .andThen(grabotronSubsystem.toggleCommand());
                // .andThen(grabotronSubsystem.toggleCommand());
                // float initialAngle = ahrsSubsystem.getBalanceAngle();
                // return driveSubsystem.tankDriveCmd(() -> 0.75, () -> 0.75).withTimeout(2)
                // .andThen(new WaitCommand(2))
                // .andThen(new BalanceCommand(driveSubsystem, ahrsSubsystem::getBalanceAngle,
                // initialAngle));

        }

}
