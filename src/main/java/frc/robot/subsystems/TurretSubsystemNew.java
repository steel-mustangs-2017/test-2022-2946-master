// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Old_Commands.ManualAim;

public class TurretSubsystemNew extends SubsystemBase {

  private final WPI_TalonFX turretMotor = new WPI_TalonFX(ShooterConstants.DEVICE_ID_TURRET);

  private final double kp = ShooterConstants.TURRET_P;
  private final double ki = ShooterConstants.TURRET_I;
  private final double kd = ShooterConstants.TURRET_D;


  private final PIDController Turret_PID = new PIDController(kp, ki, kd);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystemNew() {
      turretMotor.configFactoryDefault();
      turretMotor.setNeutralMode(NeutralMode.Brake);
      turretMotor.configNominalOutputForward(.1);
      turretMotor.configNominalOutputReverse(-.1);

      Turret_PID.setTolerance(ShooterConstants.ANGLE_TOLERENCE);

  }

  @Override
  public void periodic() {
    //System.out.println("Angle: " +getAngle());
    //System.out.println(turretMotor.getMotorOutputPercent());
    if(turretMotor.getMotorOutputPercent() > 0 && getAngle() > ShooterConstants.MAX_ANGLE){
      System.out.println("STOP");
      turretMotor.set(0);}
    if(turretMotor.getMotorOutputPercent() < 0 && getAngle() < ShooterConstants.MIN_ANGLE){turretMotor.set(0);}
    // This method will be called once per scheduler run
  }

  public void TurnManual(double speed){
    if(speed > 0 && getAngle() > ShooterConstants.MAX_ANGLE){
      System.out.println("STOP");
      speed  = 0;}
    if(speed < 0 && getAngle() < ShooterConstants.MIN_ANGLE){speed = 0;}
    turretMotor.set(speed);
  }
  public void StopTurn(){
    turretMotor.set(0);
  }

  public void reset(){
    turretMotor.setSelectedSensorPosition(0);
  }
  public double getPosition() {
    return turretMotor.getSelectedSensorPosition();
  }

  public double getAngle() {
    double pos = getPosition();
    return (pos/ShooterConstants.TURRET_ENCODER_TICKS); //* ShooterConstants.TURRET_RATIO;
  }
  public void setShooterAngle(double angle) {
    double ang = getAngle();
    turretMotor.set(MathUtil.clamp( Turret_PID.calculate(ang, angle), -.3 , .3));
  }

  public void AimShooter(double angle){
    double speed = MathUtil.clamp(Turret_PID.calculate(angle, 0), -.1 , .1);
    //if(angle > 0){speed = -.2;}
    //else if (angle < 0){speed = .2;}

    if (Math.abs(angle) < ShooterConstants.ANGLE_TOLERENCE){speed = 0;}
    
    if(speed > 0 && getAngle() > ShooterConstants.MAX_ANGLE){
      System.out.println("STOP");
      speed  = 0;}
    if(speed < 0 && getAngle() < ShooterConstants.MIN_ANGLE){
      System.out.println("STOP");
      speed = 0;}
     // System.out.println("SPEED: " + speed);
    TurnManual(speed);
  } 

    
  }


