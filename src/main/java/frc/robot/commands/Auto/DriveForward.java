// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ChassisSubsystem;

public class DriveForward extends CommandBase {

  private final ChassisSubsystem chassisSubsystem;
  private final double speed;

  /** Creates a new DriveForward. */
  public DriveForward(double speed, ChassisSubsystem chassisSubsystem) {
    this.chassisSubsystem = chassisSubsystem;
    this.speed = speed;

    addRequirements(chassisSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    chassisSubsystem.Drive(-speed, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassisSubsystem.Drive(-speed, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSubsystem.DriveStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
