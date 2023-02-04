// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.EncoderDriveCommand;
import frc.robot.commands.TurretSpinCommand;
import frc.robot.subsystems.AHRSSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
  // The robot's subsystems and commands are defined here...
  private final CommandXboxController driverController = new CommandXboxController(
      ControllerConstants.DRIVER_PORT);
  private final CommandXboxController coDriverController = new CommandXboxController(
      ControllerConstants.CODRIVER_PORT);
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final AHRSSubsystem ahrsSubsystem = new AHRSSubsystem();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure driving command
    driveSubsystem.setDefaultCommand(
        driveSubsystem.tankDriveCmd(() -> -driverController.getLeftY(), () -> -driverController.getRightY()));
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
    // driverController.a().whileTrue(balanceCommand(getBalanceAngle()));
    driverController.a().whileTrue(new EncoderDriveCommand(driveSubsystem, 5));
    driverController.b().onTrue(new InstantCommand(driveSubsystem::resetEncoders));

    // Co-Driver

    coDriverController.a().whileTrue(new TurretSpinCommand(new WPI_TalonFX(1), new SlewRateLimiter(.5, -.5, .25)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    SmartDashboard.putNumber("Auto Speed", 0);
    float initialAngle = ahrsSubsystem.getBalanceAngle();
    return driveSubsystem.tankDriveCmd(() -> 0.5, () -> 0.5).withTimeout(2)
        .andThen(new WaitCommand(1.0))
        .andThen(new BalanceCommand(driveSubsystem, ahrsSubsystem::getBalanceAngle, initialAngle));
  }
}
