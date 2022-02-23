package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
public class IndexerSubsystem extends SubsystemBase{
    private final WPI_TalonSRX indexerTop = new WPI_TalonSRX(IndexerConstants.IndexerTop);
    private final WPI_TalonSRX indexerBot = new WPI_TalonSRX(IndexerConstants.IndexerBot);
    private final WPI_TalonSRX indexerFeeder = new WPI_TalonSRX(IndexerConstants.IndexerFeeder); 
    private final double speed = 1; 
    public IndexerSubsystem() {
        indexerTop.configFactoryDefault();
        indexerBot.configFactoryDefault();
        indexerFeeder.configFactoryDefault();
    
        indexerBot.setInverted(false);
        indexerTop.setInverted(true);

        indexerTop.setNeutralMode(NeutralMode.Brake);
        indexerFeeder.setNeutralMode(NeutralMode.Brake);
        indexerBot.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run     
    }
    public void RunIndex() { 
    indexerTop.set(speed);
    indexerBot.set(speed);
    
    } 

    /*public void RumbleIndex() {
        indexerTop.set(speed);
        indexerBot.set(-speed);
        indexerFeeder.set(speed);
    }*/

    public void StopIndex() {
        indexerTop.set(0);
        indexerBot.set(0);
        indexerFeeder.set(0);
    }
    public void RunIndexerFeeder() {
        indexerFeeder.set(-speed);
    }
    public void StopIndexerFeeder() {
        indexerFeeder.set(0);
    }

   
}
