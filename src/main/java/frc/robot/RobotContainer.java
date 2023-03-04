// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.AutoScoreCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.EncoderDriveCommand;
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
        private final DriveSubsystem driveSubsystem = new DriveSubsystem();
        private final AHRSSubsystem ahrsSubsystem = new AHRSSubsystem();
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

                SmartDashboard.putData("Zero Arm Angle", (Sendable) armSubsystem.zeroTheCounter());
                SmartDashboard.putData("Toggle Arm Limits", (Sendable) armSubsystem.toggleEncoderLimiting());
                SmartDashboard.putData("Up To Zone", (Sendable) armSubsystem.raiseArmToZone());
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
                driveSubsystem.setDefaultCommand(driveSubsystem.tankDriveCmd(() -> driverController.getLeftY(),
                                () -> driverController.getRightY()));
                // driverController.a().whileTrue(new EncoderDriveCommand(driveSubsystem, 5));
                // driverController.b().onTrue(new
                // InstantCommand(driveSubsystem::resetEncoders));

                // Co-Driver
                armSubsystem.setDefaultCommand(armSubsystem.moveCmd(() -> -coDriverController.getLeftY()));
                coDriverController.y().whileTrue(telescopeSubsystem.moveCmd(() -> 1.0));
                coDriverController.a().whileTrue(telescopeSubsystem.moveCmd(() -> -1.0));
                coDriverController.rightTrigger(0.5).onTrue(grabotronSubsystem.toggleCommand());
                turretSubsystem.setDefaultCommand(turretSubsystem.runCmd(() -> coDriverController.getRightX()));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return Commands.runOnce(() -> {
                        float initialAngle = ahrsSubsystem.getBalanceAngle();
                        driveSubsystem.tankDriveCmd(() -> 0.75, () -> 0.75).withTimeout(2)
                                        .andThen(new WaitCommand(.2))
                                        .andThen(new BalanceCommand(driveSubsystem, ahrsSubsystem::getBalanceAngle,
                                                        initialAngle))
                                        .schedule();
                });
        }
}