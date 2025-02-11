package frc.robot.subsystems.gripperArm;

import com.ctre.phoenix6.controls.MotionMagicVoltage;

public class GripperArmConstants { // TODO: fIND ALL VALUES
    public static final int MOTOR_ID = 20; 
    public static final int LIMIT_SWITCH_ID = 9; 

    // control values 
    public static final double RESET_POWER = 0.5;
    public static final double MINIMUN_POSITION_ERROR = 1;
    public static final double MINIMUN_VELOCITY = 1;

        // MotionMagic Constants
    //TODO find values
    public static final double KP = 0;
    public static final double KD = 0;
    public static final double KS  = 0;
    public static final double KA = 0;
    public static final double KV = 0;

    public static final double PEAK_VOLTAGE = 11.5;
    public static final double PEAK_CURRENT = 60;

    public static final double FOWORD_LIMIT = 180; 
    public static final double BACKWARD_LIMIT = -180;
    public static final double LIMIT_SWITCH_POSITION = -180; // in degree

    public static final double GEAR_RATIO = 15;
    public static final double ABS_SENSOR_POSITION = 0;

    //Angles
    public static final double REEF_ANGLE = 0;
    public static final double FLOOR_ANGLE = 0;
    public static final double PROCESSOR_ANGLE = 0; //TODO: Find values

}
