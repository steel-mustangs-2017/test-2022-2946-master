package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    
    private static final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(IntakeConstants.intakeMotor_ID);
    private static final WPI_TalonFX intakeMove = new WPI_TalonFX(IntakeConstants.intake_move);
    
    public IntakeSubsystem(){

        intakeMotor.configFactoryDefault();

        intakeMotor.setInverted(false);

        intakeMove.configFactoryDefault();

        




    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run

    }
    public void RunIntake(){
        intakeMotor.set(1
        );
       System.out.println("Intake In ON");
    }

    public void StopIntake(){
        intakeMotor.set(0);
       // System.out.println("Intake Out ON");
    }

    public void ReverseIntake(){
        intakeMotor.set(-5);
        //System.out.println("Intake Out ON");
    }
    public void Intakeup(){
        intakeMove.set(-.25);
    }

    public void IntakeDown(){
        intakeMove.set(0.25);
    }

    public void IntakeStopDeploy(){
        intakeMove.set(0);
    }

}
