package frc.robot.commands.Control_Commands;


import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ControlerConstants;
import frc.robot.subsystems.ClimberSubsytem;




import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberCommand extends CommandBase {
    
    private final ClimberSubsytem climberSubsytem;
    private Joystick operatorController;

    

    public ClimberCommand(ClimberSubsytem climberSubsytem, Joystick operatorController) {
        
        this.climberSubsytem = climberSubsytem;
        this.operatorController  = operatorController;
    
        addRequirements(climberSubsytem);
        climberSubsytem.StopClimb();
        climberSubsytem.StopPivot();

    }

    @Override 
    public void execute() {


      double climbaxis = -operatorController.getRawAxis(ControlerConstants.CLIMBER_MAIN_ID);
     // double pivotaxis = -operatorController.getRawAxis(ControlerConstants.CLIMBER_PIVOT_ID);
      
     
       if(Math.abs(climbaxis) < .15){
        climbaxis = 0;
      } 
     
      


       climberSubsytem.ClimbUp(climbaxis);
    

    }


    @Override
  public void initialize() {
    climberSubsytem.StopClimb();
    climberSubsytem.StopPivot();
  }
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsytem.StopClimb();
    climberSubsytem.StopPivot();
  }
  

    

    


    
}
