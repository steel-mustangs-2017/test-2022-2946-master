// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class ChassisConstants{
        public final static int FrontRight_ID = 1;
        public final static int BackRight_ID = 0;
        public final static int FrontLeft_ID = 2;
        public final static int BackLeft_ID = 3;
    }
    public final class IntakeConstants{
        public final static int intakeMotor_ID = 10
        ;
        public final static int intake_move = 19;
        public final static double timer_second_down = 2;
        public final static double timer_second_up = 2;
    }

    /*public final class LimelightContstants {
        public static final double 
    }*/
public final static class ShooterConstants {
    public static final int DEVICE_ID_SHOOTER_RIGHT = 4;
    public static final int DEVICE_ID_SHOOTER_LEFT = 5;
    public static final int DEVICE_ID_TURRET = 9;

    

        public final static double SHOOT_P = .6;
        public final static double SHOOT_I = 0;
        public final static double SHOOT_D = 20;

        public final static double SHOOT_ANGLE = 70;
        public final static double SHOOT_DELTA_H = 3.3;

        public final static double SHOOTER_ENCODER_TICKS = 2048/600;
        public static final double EFFICENCY = .33;

        //////////////////////////////////////////////////////////

        public final static double TURRET_P = 0.05;
        public final static double TURRET_I = 0;
        public final static double TURRET_D = 0;

        public final static double TURRET_RATIO = (1/5)*(36/200);
        public final static int TURRET_ENCODER_TICKS = 125;
        public static final double ANGLE_TOLERENCE = 1.75;
        public static final double MAX_ANGLE = 100; //100
        public static final double MIN_ANGLE = -180; //180
    
}
    // All BAG motors
    public final static class IndexerConstants {
        public final static int IndexerTop = 8;
        public final static int IndexerBot = 7;
        public final static int IndexerFeeder = 6;

    }

    public final static class LimelightConstants {
        public static final double LIMELIGHT_KP = .01;
        public static final double LIMELIGHT_KI = 0;
        public static final double LIMELIGHT_KD = 0;

        public final static double L_HEIGHT = 2.92;
        public final static double L_ANGLE = 25.75;
        public final static double H_HEIGHT = 8.6666666;
    }

public final static class ClimberConstants {
    public static final int PivotArmLeft = 14;
    public static final int PivotArmRight = 15;
    public static final int ClimberArmLeft = 16;
    public static final int ClimberArmRight = 17; 


}

    public final static class ControlerConstants {
        public static final int PORT_ID_OPERATOR_CONTROLER = 1;
        public static final int PORT_ID_DRIVER_CONTROLLER = 0;
        public static final int Aim_Axis_ID = 0;
        public static final int Camera_ON_BUTTON_ID = 7;
        public static final int IndexerFeeder_Button_LB_ID = 5;

        public static final int SHOOT_AXIS_2_ID = 3;
        public static final int SHOOT_BUTTON_RB_ID = 6;
    

        

        public static final int CONTROLLER_BUTTON_A_ID = 1;
        public static final int CONTROLLER_BUTTON_B_ID = 2;
        public static final int CONTROLLER_BUTTON_X_ID = 3;
        public static final int CONTROLLER_BUTTON_Y_ID = 4;
        public static final int Controller_Button_Start_ID = 8;



        public static final int CLIMBER_MAIN_ID = 5; // Right Stick Y-Axis
        public static final int CLIMBER_PIVOT_ID = 4; // Right Stick X-Axis

        public static final int SHOOTER_VISION_START_ID = 2; //Button B
        public static final int TURRET_MANUAL_AXIS_ID = 0; // Left Stick X-Axis
        public static final int INDEXER_FEEDER_ID = 3; //Button X
        public static final int INDEXER_MANUAL_ID = 5; //Left Bumper

        public static final int INTAKE_OUT_ID = 4; //Button Y
        public static final int INTAKE_IN_ID = 1; //Button A
        public static final int SHOOTER_MANUAL_AXIS_ID = 3; //Right Trigger


        //////////////////////////////////////////////////////////////////////////
        public static final int FORWARD_AXIS_ID = 1;
        public static final int TURN_AXIS_ID = 5;
        public static final int FORWARD_THROTTLE_AXIS_ID = 2;
        public static final int TURN_THROTTLE_AXIS_ID = 6;
        
       
        
    }        

    public static final class AutoConstants{

        public static final double DEPLOY_TIME = .8;
        public static final double REVERSE_TIME = 3;
        public static final double SHOOT_TIME = 3;
        public static final double FORWARD_TIME = 2;

    }
}
    


