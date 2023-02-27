package frc.robot.commands;


public class AutoScoreCommand extends CommandBase {
    ArmSubsystem armSubsystem = new ArmSubsystem();
    GRABOTRONSubsystem grabotronSubsystem = new GRABOTRONSubsystem();
    PIDController pidController = new PIDController(0, 0, 0,);

    public AutoScoreCommand() {
      this.armSubsystem = armSubsystem;
      this.grabotronSubsystem = grabotronSubsystem;



    }
    @Override
    public void initialize() {
          armSubsystem.getEncoder().reset;

    }


    
    @Override
    public void execute() {
        
    }






}