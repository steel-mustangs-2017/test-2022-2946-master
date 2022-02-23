package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;




public class TurretSubsystem extends SubsystemBase {
   
    private final WPI_TalonFX turretMotor = new WPI_TalonFX(ShooterConstants.DEVICE_ID_TURRET);
    private boolean Limit = false;
    private boolean init = false;

    public TurretSubsystem() {
     turretMotor.configFactoryDefault();
     turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
     turretMotor.setSensorPhase(true);
     turretMotor.configNominalOutputForward(.1);
     turretMotor.configNominalOutputReverse(-.1);
     reset();
    }



    @Override 
    public void periodic() {

    }
    public void Turn(double speed) {
        if(Limit){
            System.out.println("limit" + getPosition());
            if(speed < 0 && getPosition() > 25000){speed = 0;System.out.println("Limit Right");}
            else if (speed < 0 && getPosition() < -100000){speed = 0; System.out.println("limit Left");}
        }
        turretMotor.set(speed);
    }
    public void StopTurn(){
        turretMotor.set(0);
    }
    public void reset(){
        turretMotor.setSelectedSensorPosition(0);
        Limit = false;
    }
    public double getPosition() {
        return turretMotor.getSelectedSensorVelocity();
    }
    public void LimitInit(){
        Limit = true;
        turretMotor.configForwardSoftLimitThreshold(240);
        turretMotor.configReverseSoftLimitThreshold(-180);
        turretMotor.configReverseSoftLimitEnable(true);
        turretMotor.configForwardSoftLimitEnable(true);
    }
    public void Init(){
        init = true;
    }
    public boolean getInit(){return init;}

   /* public void TurretSpinRight(){
        final double speed = 0.3;
       turretMotor.set(speed);
       }
       public void TurretSpinLeft(){
        final double speed = -0.3;
       turretMotor.set(speed);
       }*/

}
