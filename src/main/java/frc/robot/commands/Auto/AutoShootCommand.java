// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystemNew;
import frc.robot.subsystems.TurretSubsystemNew;

public class AutoShootCommand extends CommandBase {

  private final LimelightSubsystem limelightSubsystem;
  private final TurretSubsystemNew turretSubsystemNew ;
  private final ShooterSubsystemNew shooterSubsystemNew;
  private final IndexerSubsystem indexerSubsystem;

  private final Timer shootTime = new Timer();


  
  /** Creates a new Index_Shooter_Command. */
  public AutoShootCommand(ShooterSubsystemNew shooterSubsystemNew, LimelightSubsystem limelightSubsystem, 
  TurretSubsystemNew turretSubsystemNew, IndexerSubsystem indexerSubsystem) {

    this.shooterSubsystemNew = shooterSubsystemNew;
    this.limelightSubsystem = limelightSubsystem;
    this.turretSubsystemNew = turretSubsystemNew;
    this.indexerSubsystem = indexerSubsystem;


    addRequirements(shooterSubsystemNew,limelightSubsystem,turretSubsystemNew,indexerSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();

    limelightSubsystem.setVision();
    limelightSubsystem.setLedOn();

    shooterSubsystemNew.StopShoot();

    turretSubsystemNew.StopTurn();
    
    indexerSubsystem.StopIndex();
    turretSubsystemNew.reset();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("SHOOT");
    super.execute();

      
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
                  
                  shootTime.stop();
                  shootTime.reset();
                }
                else if(shootTime.get() == 0){
                  shootTime.start();
                }
              
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
System.out.println("Shoot End");
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
