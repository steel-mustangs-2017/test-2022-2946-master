// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ChassisConstants;

public class ChassisSubsystem extends SubsystemBase {

  private final WPI_TalonFX driveFrontRight = new WPI_TalonFX(ChassisConstants.FrontRight_ID);
  private final WPI_TalonFX driveBackRight = new WPI_TalonFX(ChassisConstants.BackRight_ID);
  private final MotorControllerGroup driveRight = new MotorControllerGroup(driveFrontRight, driveBackRight);

  private final WPI_TalonFX driveFrontLeft = new WPI_TalonFX(ChassisConstants.FrontLeft_ID);
  private final WPI_TalonFX driveBackLeft = new WPI_TalonFX(ChassisConstants.BackLeft_ID);
  private final MotorControllerGroup driveLeft = new MotorControllerGroup(driveFrontLeft, driveBackLeft);

  private final DifferentialDrive _Drive = new DifferentialDrive(driveRight,driveLeft);
  /** Creates a new Chassis. */
  public ChassisSubsystem() {

    driveFrontRight.configFactoryDefault();
      driveBackRight.configFactoryDefault();

      driveFrontLeft.configFactoryDefault();
      driveBackLeft.configFactoryDefault();

      driveFrontRight.setNeutralMode(NeutralMode.Brake);
      driveBackRight.setNeutralMode(NeutralMode.Brake);

      driveFrontLeft.setNeutralMode(NeutralMode.Brake);
      driveBackLeft.setNeutralMode(NeutralMode.Brake);

      driveRight.setInverted(false);
      driveLeft.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void Drive(Double forward, Double turn) {
   _Drive.arcadeDrive(forward, turn);
   
  }
  public void DriveRight(double speed){
    driveLeft.set(speed);
  }
  public void DriveStop() {
    _Drive.arcadeDrive(0, 0);
  }
}
