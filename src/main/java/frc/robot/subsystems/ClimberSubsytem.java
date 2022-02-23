package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimberSubsytem extends SubsystemBase{

    static WPI_TalonFX climberright = new WPI_TalonFX(ClimberConstants.ClimberArmRight);
    static WPI_TalonFX climberleft = new WPI_TalonFX(ClimberConstants.ClimberArmLeft);
    static WPI_TalonFX PivotRight = new WPI_TalonFX(ClimberConstants.PivotArmRight);
    static WPI_TalonFX PivotLeft = new WPI_TalonFX(ClimberConstants.PivotArmLeft);

    public ClimberSubsytem() {
        climberright.setInverted(true);
        climberleft.setInverted(false);
        
        PivotRight.setInverted(true);
        PivotLeft.setInverted(false);

        climberright.setNeutralMode(NeutralMode.Brake);
        climberleft.setNeutralMode(NeutralMode.Brake);

        PivotLeft.setNeutralMode(NeutralMode.Brake);
        PivotRight.setNeutralMode(NeutralMode.Brake);

    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
  }
   public void StopClimb() {
    climberleft.set(0);
    climberright.set(0);
   }
    public void ClimbUp(double speed) {
    climberleft.set(speed);
    climberright.set(speed);
   }
    public void ClimbDown(double speed) {
    climberleft.set(-speed);
    climberright.set(-speed);
    }

    public void StopPivot() {
        PivotLeft.set(0);
        PivotRight.set(0);
      }
      public void Pivotforward(double speed) {
        PivotLeft.set(-speed/4);
        PivotRight.set(-speed/4);
      }
      public void Pivotdown(double speed) {
        PivotLeft.set(-speed/4);
        PivotRight.set(-speed/4);
      }
     
}

