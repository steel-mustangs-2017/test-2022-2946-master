// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Control_Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControlerConstants;
import frc.robot.subsystems.ChassisSubsystem;

public class DriveCommand extends CommandBase {
  private final ChassisSubsystem chassisSubsystem;
  private final Joystick driverController;
  private double forward;
  private double turn;
  private double forwardThrottle;
  private double turnThrottle;
  /** Creates a new DriveCommand. */
  public DriveCommand(Joystick driverController, ChassisSubsystem chassisSubsystem) {
    this.driverController = driverController;
    this.chassisSubsystem = chassisSubsystem;

    addRequirements(chassisSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    chassisSubsystem.DriveStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forwardThrottle = 1-driverController.getRawAxis(ControlerConstants.FORWARD_THROTTLE_AXIS_ID);
    turnThrottle = (driverController.getRawAxis(ControlerConstants.TURN_THROTTLE_AXIS_ID)+1)/4+.6;

    forward = driverController.getRawAxis(ControlerConstants.FORWARD_AXIS_ID);
    turn = driverController.getRawAxis(ControlerConstants.TURN_AXIS_ID) * 0.5 * turnThrottle;

    if (Math.abs(forward) < 0.05) {
      forward = 0;
    }

    forward = forward * forwardThrottle;

    chassisSubsystem.Drive(forward, turn);
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