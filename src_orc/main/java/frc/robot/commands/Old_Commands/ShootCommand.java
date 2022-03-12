// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Old_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends CommandBase {

  private final ShooterSubsystem shooterSubsystem;
  private final Joystick operatorController;
  private double HighShoot;
  private boolean LowShoot;
  private final IndexerSubsystem indexerSubsystem;
  /** Creates a new ShootCommand. */
  public ShootCommand(Joystick operatorController, ShooterSubsystem shooterSubsystem,IndexerSubsystem indexerSubsystem) {
      this.shooterSubsystem = shooterSubsystem;
      this.operatorController = operatorController;
      this.indexerSubsystem = indexerSubsystem;

  
    addRequirements(shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexerSubsystem);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize(){
    super.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    super.execute();
    HighShoot = operatorController.getRawAxis(ControlerConstants.SHOOT_AXIS_2_ID);
    //System.out.println("HighShoot = " + HighShoot);
    LowShoot = operatorController.getRawButton(ControlerConstants.SHOOT_BUTTON_RB_ID);
    boolean Indexerfeeder = operatorController.getRawButton(ControlerConstants.IndexerFeeder_Button_LB_ID);
    if(HighShoot>0.1){
      shooterSubsystem.ShootHigh();
      while(shooterSubsystem.AtShootHighVelocity() == false){
        indexerSubsystem.StopIndex();
      }
      indexerSubsystem.RunIndex();
    
    }
    else if(LowShoot){
     //System.out.println("LowShoot = " + LowShoot);
      shooterSubsystem.Shootlow();
     while(shooterSubsystem.AtShootLowVelocity() == false){
       indexerSubsystem.StopIndex();
      }
      indexerSubsystem.RunIndex();
      
    }
    else{
      shooterSubsystem.StopShoot();
      indexerSubsystem.StopIndex();
    }
    //shooterSubsystem.StopShoot();
    //indexerSubsystem.Stopindex();
 
    if(Indexerfeeder){
    indexerSubsystem.RunIndexerFeeder();
    }
    else {
      indexerSubsystem.StopIndexerFeeder();
    }
  }
  

  
    
   
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    shooterSubsystem.StopShoot();
    indexerSubsystem.StopIndex();
  }

  // Returns true when the command should end.
 
  public boolean isFinished() {
    return false;
  }
}
