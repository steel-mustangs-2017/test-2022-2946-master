package frc.robot.commands.Old_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class TurretInitCommand extends CommandBase {
   private final TurretSubsystem turretSubsystem;

   private final double tolerance = 1000;
   private boolean atPoint = false;

  private final double target = 112000;

  /** Creates a new TurretInitCommand. 
   * @param limelightSubsystem*/
  public TurretInitCommand(TurretSubsystem turretSubsystem, LimelightSubsystem limelightSubsystem) {
    this.turretSubsystem = turretSubsystem;

    addRequirements(turretSubsystem);
    }

    @Override 
    public void initialize() {
       super.initialize();
       turretSubsystem.reset();

    }

    @Override
    public void execute() {
      super.execute();
    PIDController pid = new PIDController(ShooterConstants.TURRET_P, 0.00000005, ShooterConstants.TURRET_I);
    pid.setTolerance(tolerance);
    
    System.out.println("hi: " +MathUtil.clamp(pid.calculate(target, turretSubsystem.getPosition()), -.75, .75));
    turretSubsystem.Turn(MathUtil.clamp(pid.calculate(target, turretSubsystem.getPosition()), -.75, .75)); 
 
    
    atPoint = pid.atSetpoint();
    pid.close();
   
    }

    @Override
    public void end(boolean interrupted) {
      super.end(interrupted);
      turretSubsystem.StopTurn();
      turretSubsystem.reset();
      turretSubsystem.LimitInit();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return atPoint;
    }
}
