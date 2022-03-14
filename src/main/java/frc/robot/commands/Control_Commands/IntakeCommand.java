                        package frc.robot.commands.Control_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlerConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase
{
    private final IntakeSubsystem intakeSubsystem;
    private final Joystick operatorController;
    

    public IntakeCommand(Joystick operatorController, IntakeSubsystem intakeSubsystem){
       this.operatorController = operatorController;
       this.intakeSubsystem = intakeSubsystem;
       
       addRequirements(intakeSubsystem);

    }

    @Override
    public void initialize() {
      //intakeSubsystem.intakeoutBTN(); 
      //succ    
    }
    @Override
    public void execute() {
   // System.out.println("Intake On");
    boolean intakeinBTN = operatorController.getRawButton(ControlerConstants.CONTROLLER_BUTTON_A_ID);
    boolean intakeoutBTN = operatorController.getRawButton(ControlerConstants.CONTROLLER_BUTTON_Y_ID);
    int IntakeUp = operatorController.getPOV();
    if(IntakeUp == 180) {
      intakeSubsystem.IntakeDown();
    }
    else if(IntakeUp == 0) {
      intakeSubsystem.Intakeup();
    }
    else{
      intakeSubsystem.IntakeStopDeploy();
    }

    if(intakeinBTN){
      intakeSubsystem.RunIntake();
    }
    else if (intakeoutBTN){
      intakeSubsystem.ReverseIntake();
    }
    else{intakeSubsystem.StopIntake();}


    }

    @Override
    public void end(boolean interrupted) {
      intakeSubsystem.StopIntake();
      intakeSubsystem.IntakeStopDeploy();
    }
    

    
    
}
