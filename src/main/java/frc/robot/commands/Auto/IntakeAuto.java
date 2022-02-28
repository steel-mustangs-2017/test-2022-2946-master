package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends CommandBase {
    
    private IntakeSubsystem intakesubsystem = new IntakeSubsystem();
    
    public IntakeAuto(IntakeSubsystem intakeSubsystem) {
        this.intakesubsystem = intakesubsystem;
        
        addRequirements(intakesubsystem);
    }

    @Override 
    public void initialize(){
        intakesubsystem.RunIntake();
        
    }
    
    @Override
    public void execute() {}

    @Override 
    public void end(boolean interrupted) {
        intakesubsystem.StopIntake();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }


    
}
