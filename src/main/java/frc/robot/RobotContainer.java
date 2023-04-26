// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Robot.AutonomousChoices;
import frc.robot.commands.ArmPIDCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.EncoderDriveCommand;
import frc.robot.commands.MoveBackwardsCommand;
import frc.robot.commands.MoveForwardCommand;
import frc.robot.commands.NewBalanceCommand;
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

                driveSubsystem.setDefaultCommand(
                        driveSubsystem.arcadeDriveCmd( 
                                // ()-> -driverController.getLeftY(),
                                // ()-> -driverController.getRightX()));
                                () -> {
                                        if (driverController.rightTrigger().getAsBoolean()) {
                                                return 0.5 * -driverController.getLeftY();
                                        } else {
                                                return -driverController.getLeftY();
                                        }

                        },  () -> {
                                if (driverController.leftTrigger().getAsBoolean()) {
                                        return 0.5 * -driverController.getLeftY();
                                } else {
                                        return -driverController.getRightX();
                                }

                                       
                        }));

                           


                // Co-Driver
                armSubsystem.setDefaultCommand(armSubsystem.moveCmd(() -> -coDriverController.getLeftY()));
                 //make turret work
                turretSubsystem.setDefaultCommand(turretSubsystem.runCmd(() -> coDriverController.getRightX()));
                //telescope out and in
                coDriverController.y().whileTrue(telescopeSubsystem.moveCmd(() -> 1.0));
                coDriverController.a().whileTrue(telescopeSubsystem.moveCmd(() -> -1.0));
                //toggle grabber
                coDriverController.rightTrigger(0.5).onTrue(grabotronSubsystem.toggleCommand());
                //reset encoders
                coDriverController.povLeft().onTrue(Commands.parallel(telescopeSubsystem.resetEncodersCommand(), armSubsystem.resetEncodersCommand()));

                //------------------------------------------------------------------------------------------------------------------------
                //------------------------------------------------------------------------------------------------------------------------

                //reset telescope position
                coDriverController.x().onTrue(new TelescopePIDCommand(telescopeSubsystem, 0));

                //reset arm and telescope position to default
                coDriverController.rightBumper().onTrue(new TelescopePIDCommand(telescopeSubsystem, 0).andThen(new ArmPIDCommand(armSubsystem, -70))); 

                //pick up from ground preset
                
                coDriverController.leftTrigger().onTrue(new ArmPIDCommand(armSubsystem, -47.46).andThen(new TelescopePIDCommand(telescopeSubsystem, 1.05)).andThen(grabotronSubsystem.toggleCommand()));

                //score mid preset
              
                coDriverController.b().onTrue(new ArmPIDCommand(armSubsystem, 9.5).andThen(new TelescopePIDCommand(telescopeSubsystem, 1.05)));
                
                //score high preset(theoretical)
                //coDriverController.povUp().onTrue(new ArmPIDCommand(armsubsystem, 22).andThen(new TelescopePIDCommand(telescopeSubsystem, 5.5)));

                //substation pickup preset
                coDriverController.leftBumper().onTrue(new TelescopePIDCommand(telescopeSubsystem, 0.05).andThen(new ArmPIDCommand(armSubsystem, 17.5)));
                
                
               
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

        //Auto that goes hee hee we have good alliance members and they want us to do absolutely nothing except score low
        private final Command bumpAlone = new MoveForwardCommand(driveSubsystem).withTimeout(0.5);

        PathPlannerTrajectory twoPieceTest = PathPlanner.loadPath("TwoGamePieceAutoL", 1, 0.5);
        PathPlannerTrajectory twoPieceTest2 = PathPlanner.loadPath("TwoGamePieceAuto2", 1, 0.5);

        //Maybe try to get this to work? good for programmers to learn how to use pathplanner correctly
        //Also probably ask Reza how to make this work from scratch without anything in drivesubsystem
        //cus what I have there is prob wrong no cap

        private final Command TWO_PIECE_AUTO = new MoveForwardCommand(driveSubsystem).withTimeout(0.5)
                        .andThen(driveSubsystem.followTrajectoryCommand(twoPieceTest, true, false))
                        .andThen(new ArmPIDCommand(armSubsystem, -50))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.005))
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 0.005))
                        .andThen(driveSubsystem.followTrajectoryCommand(twoPieceTest2, false, false))
                        .andThen(new ArmPIDCommand(armSubsystem, 18))
                        .andThen(new TelescopePIDCommand(telescopeSubsystem, 3.5)) // Score Mid
                        .andThen(grabotronSubsystem.toggleCommand())
                        .andThen(Commands.parallel(new TelescopePIDCommand(telescopeSubsystem, 0.05),
                                        grabotronSubsystem.toggleCommand()))
                        .andThen(new ArmPIDCommand(armSubsystem, -69));

        private final Command BALANCE = 
        //un-comment next comment if u want it to score low and then balance(completely untested lol)
        
        //new MoveForwardCommand(driveSubsystem).withTimeout(0.5).andThen(

                        new MoveBackwardsCommand(driveSubsystem).withTimeout(3.4)
                                        .andThen(new NewBalanceCommand(driveSubsystem, ahrsSubsystem::getBalanceAngle));
                                        // .andThen(new BalanceCommand(driveSubsystem, ahrsSubsystem::getBalanceAngle,
                                        //                 0));


        //Btw whoever is reading this, figure out optimal values for maxVelocity and maxAcceleration cus thats prob important
        PathPlannerTrajectory ScoreHigh = PathPlanner.loadPath("ScoreHigh", 1, 0.5);
        PathPlannerTrajectory ScoreHigh2 = PathPlanner.loadPath("ScoreHigh2", 1, 0.5);

       
                       
        //worked like half of the time on bump side :( but like literally always worked on non-bump
        private final Command BumpBackOut = new MoveForwardCommand(driveSubsystem).withTimeout(0.5)
                        .andThen(new MoveBackwardsCommand(driveSubsystem).withTimeout(2.5));

        PathPlannerTrajectory NoArmlol = PathPlanner.loadPath("HeeHeeNoArmAuto", 1, 0.5);
        PathPlannerTrajectory NoArmlol2 = PathPlanner.loadPath("HeeHeeNoArmAuto2", 1, 0.5);

        //ignore this, I was testing to see if I could make it drive straight backwards and didn't bother to change the name
        private final Command BUMP_SCORE_SECOND_PIECE = new MoveBackwardsCommand(driveSubsystem).withTimeout(2.4);
                        

        private final Command DEFAULT = Commands.runOnce(() -> {
                System.out.println(":)");

        });

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand(AutonomousChoices autoCode) {
                switch (autoCode) {
                        case BumpAlone:
                                return bumpAlone;

                        case TwoPieceAuto:
                                return TWO_PIECE_AUTO;

                        case Balance:
                                return BALANCE;
                        case BumpBackOut:
                                return BumpBackOut;
                        case BumpScoreSecondPiece:
                                return BUMP_SCORE_SECOND_PIECE;
                        default:
                                return DEFAULT;
                }
        }

}