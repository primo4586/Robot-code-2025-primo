package frc.robot.subsystems.gripperArm;

public class GripperArmConstants {
    public static final int MOTOR_ID = 20; 
    public static final int LIMIT_SWITCH_ID = 9; 

    // control values 
    public static final double RESET_POWER = 0.25;
    public static final double MINIMUN_POSITION_ERROR = 1;
    public static final double MINIMUN_VELOCITY = 1;

        // PID Constants
    public static final double KP = 19.716;
    public static final double KD = 0.16079;
    public static final double KS  = 0;
    public static final double KA = 0.015145;
    public static final double KV = 1.6843;

    public static final double PEAK_VOLTAGE = 11.5;
    public static final double PEAK_CURRENT = 60;

    public static final double FOWORD_LIMIT = 180; 
    public static final double BACKWARD_LIMIT = -180;
    public static final double LIMIT_SWITCH_POSITION = -180; // in degree

    public static final double GEAR_RATIO = 15;
    public static final double ABS_SENSOR_POSITION = 0;

    public static final double VELOCITY = 20;
    public static final double ACCELERATION = 20/3;

    //Angles
    public static final double REEF_ANGLE = -2.5; //if this value causes problems then raise this value by very little
    public static final double FLOOR_ANGLE = -2.6;
    public static final double PROCESSOR_ANGLE = -1.8;
    

}
