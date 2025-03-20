package frc.robot.subsystems.Disposer;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class DisposerConstants {
    public static final int MOTOR_ID = 20; 

    // control values 
    public static final double RESET_POWER = 0.35;
    public static final double MINIMUN_POSITION_ERROR = 1;
    public static final double MINIMUN_VELOCITY = 1;

        // PID Constants
    public static final double KP = 20;
    public static final double KD = 0;
    public static final double KS  = 0;

    public static final int PEAK_CURRENT = 30;

     public static final IdleMode NEUTRAL_MODE = IdleMode.kCoast;
    public static final Boolean INVERTED = true;



}
