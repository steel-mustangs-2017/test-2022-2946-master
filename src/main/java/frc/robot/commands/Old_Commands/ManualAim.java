package frc.robot.commands.Old_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.ControlerConstants;;

public class ManualAim extends CommandBase{
    private final TurretSubsystem turretSubsystem;
    private final Joystick operatorController;
    //to change speed change divison instead IDIOT!!!
    private double turn = 1;
    

    public ManualAim(Joystick operatorController, TurretSubsystem turretSubsystem) {
        this.turretSubsystem = turretSubsystem;
        this.operatorController = operatorController;

        addRequirements(turretSubsystem);
    
    }

    @Override 
    public void initialize() {
        super.execute(); 
    }

        @Override 
        public void execute() {
            super.execute();
            turn = operatorController.getRawAxis(ControlerConstants.Aim_Axis_ID);
            //System.out.println("turn1 = " + turn);
            if(Math.abs(turn) < .5){turn = 0;}
            
             //System.out.println("turn2 = " + turn);
            turretSubsystem.Turn(turn/4);
        }
 

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        turretSubsystem.StopTurn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
