// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Control_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.sql.Time;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.ControlerConstants;
import frc.robot.Constants.IntakeConstants;

public class intake1button extends CommandBase {
  /** Creates a new intake1button. */

  private final IntakeSubsystem intakeSubsystem;
  private final Joystick operatorController;
  private final Timer timerDown =new Timer();
  private final Timer timerUp =new Timer();
  private  boolean up = true;
  public intake1button(Joystick operatorController, IntakeSubsystem intakeSubsystem ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.operatorController = operatorController;
    this.intakeSubsystem = intakeSubsystem;


    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();
      if(operatorController.getRawButtonPressed(ControlerConstants.CONTROLLER_BUTTON_A_ID)){
          intakeSubsystem.IntakeDown();
          timerDown.reset();
          timerDown.start();               
      }
      if(operatorController.getRawButton(ControlerConstants.CONTROLLER_BUTTON_A_ID)){
        intakeSubsystem.RunIntake();
      }
      else{
        intakeSubsystem.StopIntake();
      }
      if(timerDown.hasElapsed(IntakeConstants.timer_second_down)){
        intakeSubsystem.IntakeStopDeploy();
        timerDown.stop();
        timerDown.reset();
        up = false;
      }
      if(operatorController.getRawButtonReleased(ControlerConstants.CONTROLLER_BUTTON_A_ID)){
        intakeSubsystem.Intakeup();
        timerUp.reset();
        timerUp.start();
      }
      if(timerUp.hasElapsed(IntakeConstants.timer_second_up)){
        intakeSubsystem.IntakeStopDeploy();
        up = true;
        timerUp.reset();
        timerUp.stop();
      }

      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
