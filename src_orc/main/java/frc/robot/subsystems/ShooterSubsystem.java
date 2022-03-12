// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final WPI_TalonFX shooterMotor = new WPI_TalonFX(ShooterConstants.DEVICE_ID_SHOOTER_RIGHT);
  private final WPI_TalonFX shooterMotorSlave = new WPI_TalonFX(ShooterConstants.DEVICE_ID_SHOOTER_LEFT);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  shooterMotor.configFactoryDefault();
  shooterMotorSlave.configFactoryDefault();

  shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  shooterMotorSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  
  shooterMotor.setInverted(false);
  shooterMotorSlave.setInverted(true);
  
  shooterMotor.setNeutralMode(NeutralMode.Coast);
  shooterMotorSlave.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    //System.out.println("Velocity: " + shooterMotor.getSelectedSensorVelocity());
    //System.out.println(shooterMotor.getSelectedSensorVelocity() > 20000);
    // This method will be called once per scheduler run
  }

  public void ShootHigh(){
    final double speed = 1; 
   shooterMotor.set(speed);
    shooterMotorSlave.set(speed);
    if(AtShootHighVelocity()){
      System.out.println("High At Velocity");
    }
    
  }
  public void Shootlow(){
     final double speed = 0.3;
    shooterMotor.set(speed);
    shooterMotorSlave.set(speed);
    if(AtShootLowVelocity()){
      System.out.println("Low at Velocity");
    }
    
  }
  public boolean AtShootHighVelocity(){
    return shooterMotor.getSelectedSensorVelocity() > 15000;
  }
  public boolean AtShootLowVelocity(){
    return shooterMotor.getSelectedSensorVelocity() > 5000;
  }
  public void StopShoot(){
    shooterMotor.set(0);
    shooterMotorSlave.set(0);
  }
}

 //one button