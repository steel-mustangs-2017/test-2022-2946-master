// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystemNew extends SubsystemBase {

  private final WPI_TalonFX shooterRight = new WPI_TalonFX(ShooterConstants.DEVICE_ID_SHOOTER_RIGHT);
  private final WPI_TalonFX shooterLeft = new WPI_TalonFX(ShooterConstants.DEVICE_ID_SHOOTER_LEFT);

  private final double kp = ShooterConstants.SHOOT_P;
  private final double ki = ShooterConstants.SHOOT_I;
  private final double kd = ShooterConstants.SHOOT_D;


  private final PIDController Right_PID = new PIDController(kp, ki, kd);
  private final PIDController Left_PID = new PIDController(kp, ki, kd);

  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystemNew() {
    shooterRight.configFactoryDefault();
    shooterRight.setNeutralMode(NeutralMode.Coast);
    shooterRight.setInverted(false);

    shooterLeft.configFactoryDefault();
    shooterLeft.setNeutralMode(NeutralMode.Coast);
    shooterLeft.setInverted(true);

    shooterRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    shooterRight.config_kP(0, kp);
    shooterRight.config_kI(0, ki);
    shooterRight.config_kD(0, kd);

    shooterLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);

    shooterLeft.config_kP(0, kp);
    shooterLeft.config_kI(0, ki);
    shooterLeft.config_kD(0, kd);

    Right_PID.setTolerance(30);
    Left_PID.setTolerance(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ShootManual(double speed){
    shooterRight.set(speed);
    shooterLeft.set(speed);
  }
  public void StopShoot(){
    shooterRight.set(0);
    shooterLeft.set(0);
  }

  public double getEncoderVelocity(){
    double rv = shooterRight.getSelectedSensorVelocity();
    double lv = shooterLeft.getSelectedSensorVelocity();

    return (rv+lv)/2;
  }

  public double getRPM(){
    double ev = getEncoderVelocity();
    return ev/ShooterConstants.SHOOTER_ENCODER_TICKS;
  }
  public double RPM_to_Encoder(double RPM){
    return RPM*ShooterConstants.SHOOTER_ENCODER_TICKS;
  }

  public void ShootRPM(double RPM){

    System.out.println("RPM: " + getRPM());
    shooterRight.set(ControlMode.Velocity, RPM_to_Encoder(RPM) * 1.1);
    shooterLeft.set(ControlMode.Velocity, RPM_to_Encoder(RPM) *1.1);

    
  }

  public void ShootDistance(double distance){
    ShootRPM(Dist_to_RPM(distance));
  }

  public double Dist_to_RPM(double distance){
    double a = ShooterConstants.SHOOT_ANGLE;
    double h = ShooterConstants.SHOOT_DELTA_H;
    double v = Math.sqrt(-(32.17*(distance*distance) * (1 +Math.pow(Math.tan(Math.toRadians(a)), 2)))/(2*h - 2*distance*Math.tan(Math.toRadians(a))));
    System.out.println("VEL:" + v);
    double c = 3.14159265*(4.0/12.0);

    float RPM = (((float)(v/c)) * 60) / (float)ShooterConstants.EFFICENCY;

    System.out.println("RPM3:" + RPM);

    return RPM;
  }



  public boolean At_Speed(double target){
    if(Math.abs(Dist_to_RPM(target)-getRPM()) < 200){
      return true;
    }
    else{return false;}
  }


}
