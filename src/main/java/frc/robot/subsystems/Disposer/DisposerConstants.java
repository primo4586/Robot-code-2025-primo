package frc.robot.subsystems.Disposer;

public class DisposerConstants {
    public static final int MOTOR_ID = 20; 

        // PID Constants
    public static final double KP = 10;
    public static final double KD = 0;
    public static final double KS  = 0;
    public static final double KA = 0;
    public static final double KV = 0;

    public static final double PEAK_VOLTAGE = 11.5;
    public static final double PEAK_CURRENT = 40;

    public static final double FOWORD_LIMIT = 40; 
    public static final double BACKWARD_LIMIT = 0;


    //Angles
    public static final double HOME_POSITION = 0; //if this value causes problems then raise this value by very little
    public static final double READY_POSITION = 15;
    

}
