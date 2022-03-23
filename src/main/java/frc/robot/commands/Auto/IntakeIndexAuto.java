// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.IntakeConstants;;

public class IntakeIndexAuto extends CommandBase {

  private IntakeSubsystem intakeSubsystem;
  private IndexerSubsystem indexerSubsystem;
  private final Timer timerDown =new Timer();
  private final Timer timerUp =new Timer();
  /** Creates a new IntakeIndexAuto. */
  public IntakeIndexAuto(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {

    this.indexerSubsystem = indexerSubsystem;
    this.intakeSubsystem = intakeSubsystem;

    addRequirements(intakeSubsystem, indexerSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  /*public void AutoStart(){
    intakeSubsystem.IntakeDown();
    timerDown.reset();
    timerDown.start();
    if(timerDown.hasElapsed(IntakeConstants.timer_second_down)){
      intakeSubsystem.IntakeStopDeploy();
      timerDown.stop();
      timerDown.reset();
    }
  }
*/
  public void AutoInturupt(){
    
  }
  @Override
  public void initialize() {
    intakeSubsystem.StopIntake();
    indexerSubsystem.StopIndexerFeeder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    indexerSubsystem.RunIndexerFeeder();
    intakeSubsystem.RunIntake();
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.StopIntake();
    indexerSubsystem.StopIndexerFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
