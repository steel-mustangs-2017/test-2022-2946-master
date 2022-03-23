// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ControlerConstants;
import frc.robot.commands.Auto.AutoShootCommand;
import frc.robot.commands.Auto.DriveForward;
import frc.robot.commands.Auto.IntakeIndexAuto;
import frc.robot.commands.Control_Commands.ClimberCommand;
import frc.robot.commands.Control_Commands.DriveCommand;
import frc.robot.commands.Control_Commands.IndexShooterCommand;
import frc.robot.commands.Control_Commands.intake1button;
import frc.robot.commands.Control_Commands.IntakeCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ClimberSubsytem;
import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Init Joysticks
  private final Joystick operatorController = new Joystick(ControlerConstants.PORT_ID_OPERATOR_CONTROLER);
  private final Joystick driverController = new Joystick(ControlerConstants.PORT_ID_DRIVER_CONTROLLER);
 
 
  // The robot's subsystems defined here...
  private final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final ShooterSubsystemNew shooterSubsystemNew = new ShooterSubsystemNew();
  private final ClimberSubsytem climberSubsytem = new ClimberSubsytem();
  private final TurretSubsystemNew turretSubsystemNew = new TurretSubsystemNew();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();



  private final DriveCommand driveCommand = new DriveCommand(driverController, chassisSubsystem);
  private final intake1button Intake1button = new intake1button(operatorController,intakeSubsystem);
  private final IntakeCommand intakeCommand = new IntakeCommand(operatorController,intakeSubsystem);
  private final ClimberCommand ClimberCommand = new ClimberCommand(climberSubsytem, operatorController);
  private final IndexShooterCommand indexShooterCommand  = new IndexShooterCommand(shooterSubsystemNew, limelightSubsystem, turretSubsystemNew, operatorController, indexerSubsystem);
  private final DriveForward driveForward = new DriveForward(.35, chassisSubsystem);
  private final IntakeIndexAuto intakeIndexAuto = new IntakeIndexAuto(intakeSubsystem, indexerSubsystem);
  //private final MotorTest motorTest = new MotorTest(chassisSubsystem, operatorController);
  //private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  //private final ShootCommand shootCommand = new ShootCommand(operatorController, shooterSubsystem, IndexerSubsystem);
  //private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  //private final ManualAim manualAim= new ManualAim(operatorController, turretSubsystem);
 
  //private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
 // private final TurretInitCommand turretInitCommand = new TurretInitCommand(turretSubsystem,limelightSubsystem);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureDefaultCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

   

    
    

  }


  private void configureDefaultCommands() {

    chassisSubsystem.setDefaultCommand(driveCommand);
    intakeSubsystem.setDefaultCommand(Intake1button);
    climberSubsytem.setDefaultCommand(ClimberCommand);
    shooterSubsystemNew.setDefaultCommand(indexShooterCommand);
    turretSubsystemNew.setDefaultCommand(indexShooterCommand);
    indexerSubsystem.setDefaultCommand(indexShooterCommand);
   //turretSubsystemNew.setDefaultCommand(limelightTest);
    //shooterSubsystem.setDefaultCommand(shootCommand);
    //turretSubsystem.setDefaultCommand(manualAim);
    
    //climberSubsytem.setDefaultCommand(pivotCommand);
    //limelightSubsystem.setDefaultCommand(turretInitCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return //new StartEndCommand(() -> intakeSubsystem.IntakeDown(), () -> intakeSubsystem.IntakeStopDeploy(), intakeSubsystem).withTimeout(AutoConstants.DEPLOY_TIME);
    
    new StartEndCommand(() -> intakeSubsystem.IntakeDown(), () -> intakeSubsystem.IntakeStopDeploy(), intakeSubsystem).withTimeout(AutoConstants.DEPLOY_TIME).andThen(driveForward.withTimeout(AutoConstants.REVERSE_TIME)).andThen(new AutoShootCommand(shooterSubsystemNew, limelightSubsystem, turretSubsystemNew, indexerSubsystem).withTimeout(AutoConstants.SHOOT_TIME)) 
      .andThen((new IntakeIndexAuto(intakeSubsystem, indexerSubsystem))
      .raceWith(new DriveForward(.35, chassisSubsystem)).withTimeout(AutoConstants.FORWARD_TIME))
      .andThen(new AutoShootCommand(shooterSubsystemNew, limelightSubsystem, turretSubsystemNew, indexerSubsystem).withTimeout(AutoConstants.SHOOT_TIME));
      
      
  

/*new StartEndCommand(() -> intakeSubsystem.IntakeDown(), () -> intakeSubsystem.IntakeStopDeploy(), intakeSubsystem).withTimeout(AutoConstants.DEPLOY_TIME)
.andThen(new RunCommand(() -> {intakeSubsystem.RunIntake(); indexerSubsystem.RunIndexerFeeder();}, intakeSubsystem.raceWith(
      new DriveForward(0.25,chassisSubsystem).withTimeout(AutoConstants.REVERSE_TIME))
      .andThen(
        new AutoShootCommand(shooterSubsystemNew, limelightSubsystem, turretSubsystemNew, indexerSubsystem).withTimeout(AutoConstants.SHOOT_TIME) ); 
  */
      }
      }