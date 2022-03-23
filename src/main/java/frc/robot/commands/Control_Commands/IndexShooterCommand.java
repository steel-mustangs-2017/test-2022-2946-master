// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Control_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystemNew;
import frc.robot.subsystems.TurretSubsystemNew;

public class IndexShooterCommand extends CommandBase {

  private final LimelightSubsystem limelightSubsystem;
  private final TurretSubsystemNew turretSubsystemNew ;
  private final ShooterSubsystemNew shooterSubsystemNew;
  private final IndexerSubsystem indexerSubsystem;

  private final Joystick operatorControler;

  private final Timer shootTime = new Timer();
  
  /** Creates a new Index_Shooter_Command. */
  public IndexShooterCommand(ShooterSubsystemNew shooterSubsystemNew, LimelightSubsystem limelightSubsystem, 
  TurretSubsystemNew turretSubsystemNew, Joystick operatorControler, IndexerSubsystem indexerSubsystem) {

    this.shooterSubsystemNew = shooterSubsystemNew;
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystemNew = turretSubsystemNew;
    this.operatorControler = operatorControler;
    this.indexerSubsystem = indexerSubsystem;


    addRequirements(shooterSubsystemNew,limelightSubsystem,turretSubsystemNew,indexerSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    limelightSubsystem.setCamera();
    limelightSubsystem.setLedOff();

    shooterSubsystemNew.StopShoot();

    turretSubsystemNew.StopTurn();
    
    indexerSubsystem.StopIndex();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    super.execute();

      if(operatorControler.getRawButton(ControlerConstants.SHOOTER_VISION_START_ID)){
        limelightSubsystem.setLedOn();
        limelightSubsystem.setVision();
                if(limelightSubsystem.validTarget() == 1){
                  System.out.println("Target Found");
                  
                  shooterSubsystemNew.ShootDistance(limelightSubsystem.getDistance());;
                  turretSubsystemNew.AimShooter(limelightSubsystem.getAngle());

                      if(shooterSubsystemNew.At_Speed(limelightSubsystem.getDistance()) && limelightSubsystem.getAngle() < ShooterConstants.ANGLE_TOLERENCE){
                        indexerSubsystem.RunIndex();
                        indexerSubsystem.RunIndexerFeeder();
                        turretSubsystemNew.StopTurn();
                        System.out.println("SHOOTING");
                      }
                      else{
                        shooterSubsystemNew.ShootDistance(limelightSubsystem.getDistance());
                        turretSubsystemNew.AimShooter(limelightSubsystem.getAngle());
                      }
                }

                else if (shootTime.get() > 4){
                  System.out.println("Looking For Target");
                  double turretturn = -operatorControler.getRawAxis(ControlerConstants.TURRET_MANUAL_AXIS_ID);

                  if(Math.abs(turretturn) < .25){turretturn = 0;}
                  turretSubsystemNew.TurnManual(turretturn/4);
                  if(operatorControler.getRawButton(ControlerConstants.INDEXER_MANUAL_ID)){indexerSubsystem.RunIndex();}
                  else{indexerSubsystem.StopIndex();}

                  if(operatorControler.getRawButton(ControlerConstants.INDEXER_FEEDER_ID)){indexerSubsystem.RunIndexerFeeder();}
                  else{indexerSubsystem.StopIndexerFeeder();}
                  shootTime.stop();
                  shootTime.reset();
                }
                else if(shootTime.get() == 0){
                  shootTime.start();
                }
              }
      else{
        limelightSubsystem.setLedOff();
        limelightSubsystem.setCamera();

        double turretturn = -operatorControler.getRawAxis(ControlerConstants.TURRET_MANUAL_AXIS_ID);

        if(Math.abs(turretturn) < .15){turretturn = 0;}
        turretSubsystemNew.TurnManual(turretturn/3);

        shooterSubsystemNew.ShootManual(operatorControler.getRawAxis(ControlerConstants.SHOOTER_MANUAL_AXIS_ID)/4);

        if(operatorControler.getRawButton(ControlerConstants.INDEXER_MANUAL_ID)){indexerSubsystem.RunIndex();}
        else{indexerSubsystem.StopIndex();}

        if(operatorControler.getRawButton(ControlerConstants.INDEXER_FEEDER_ID)){indexerSubsystem.RunIndexerFeeder();}
        else if(operatorControler.getRawButton(ControlerConstants.INTAKE_OUT_ID)){indexerSubsystem.ReverseIndex();}
        else{indexerSubsystem.StopIndexerFeeder();}

      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    turretSubsystemNew.StopTurn();
    shooterSubsystemNew.StopShoot();
    indexerSubsystem.StopIndex();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
