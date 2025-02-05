package frc.robot.subsystems.Gripper;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GripperConstants {
    public static final int MOTOR_ID = 22;

    public static final double COLLECT_POWER = 0.5;
    public static final double HOLED_POWER = 0.15; 
    public static final double TOSS_POWER = -0.7;
    public static final double TOSS_TIME = 0.1;

    //Peaks
    public static final double VOLTAGE_PEAK = 11.5;
    public static final double CURRENT_PEAK = 60;

    // settings
    public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    public static final double VELOCITY_MIN_ERROR = 0.1;

}
