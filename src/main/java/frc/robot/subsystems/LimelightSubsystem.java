package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new Limelight. */
  

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private boolean ledOn = true;
    private boolean  visionOn = true;

    public void  setLedOn() {
        if (!ledOn)
        {
            table.getEntry("ledMode").setNumber(3);
            ledOn = true;

        }
    }    
    public void  setLedOff() {
        if (ledOn)
        {
            table.getEntry("ledMode").setNumber(1);
            ledOn = false;

        }
    }    
    public void  setVision() {
        if (!visionOn)
        {
            table.getEntry("camMode").setNumber(0);
            visionOn = true;

        }
        
    }    
    public void  setCamera() {
        if (visionOn)
        {
            table.getEntry("camMode").setNumber(1);
            visionOn = false;

        }
    }    

    public double validTarget() {
        NetworkTableEntry tv = table.getEntry("tv");
        return tv.getDouble(0);
    }
    public double getAngle() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0.0);   
        
    }
    public double getDistance(){
        // Gets Distance in ft
  
        NetworkTableEntry ty  = table.getEntry("ty");
        //d = (h2-h1) / tan(a1+a2)
        System.out.println(ty.getDouble(0));
        double distance = (LimelightConstants.H_HEIGHT - LimelightConstants.L_HEIGHT) / Math.tan(Math.toRadians( LimelightConstants.L_ANGLE + ty.getDouble(0.0)));
  
        return distance;
    }
    
    public LimelightSubsystem(){
        //System.out.println("------------- line out");
        table.getEntry("stream").setNumber(0);
        
    }
    
    public void stream() {
        table.getEntry("stream").setNumber(0);
    }
    public void printValues () {
        

        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double validTarget = tv.getDouble(0.0);
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("valid Target", validTarget);
        SmartDashboard.putNumber("LimelightArea", area);

          
    }


  @Override
  public void periodic() {
    

    // This method will be called once per scheduler run
    setLedOff();
    setVision();
    table.getEntry("stream").setNumber(0);

    NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        double validTarget = tv.getDouble(0.0);
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("valid Target", validTarget);
        SmartDashboard.putNumber("LimelightArea", area);
  }
}

