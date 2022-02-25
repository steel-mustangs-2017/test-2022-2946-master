// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Control_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlerConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystemNew;

public class LimelightTest extends CommandBase {

  private final LimelightSubsystem limelightSubsystem;
  private final TurretSubsystemNew turretSubsystemNew;
  private final Joystick operatorControler; 
  /** Creates a new LimelightTeat. */
  public LimelightTest(LimelightSubsystem limelightSubsystem, TurretSubsystemNew turretSubsystemNew, Joystick operatorControler) {

    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystemNew = turretSubsystemNew;
    this.operatorControler = operatorControler;

    addRequirements(limelightSubsystem,turretSubsystemNew);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turretSubsystemNew.StopTurn();

    limelightSubsystem.setLedOff();
    limelightSubsystem.setCamera();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   

    if(operatorControler.getRawButton(ControlerConstants.SHOOTER_VISION_START_ID)){
      limelightSubsystem.setLedOn();
      limelightSubsystem.setVision();
      //System.out.println("ANGLE: " + limelightSubsystem.getAngle());
      System.out.println("DIST: " + limelightSubsystem.getDistance());
     
    }
    else{
      limelightSubsystem.setLedOff();
    limelightSubsystem.setCamera();
    double turretturn = operatorControler.getRawAxis(ControlerConstants.TURRET_MANUAL_AXIS_ID);

    if(Math.abs(turretturn) < .25){turretturn = 0;}
    turretSubsystemNew.TurnManual(-turretturn/4);}
    //System.out.println("TURN: " + turretturn/4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelightSubsystem.setLedOff();
    limelightSubsystem.setCamera();

    turretSubsystemNew.StopTurn();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
