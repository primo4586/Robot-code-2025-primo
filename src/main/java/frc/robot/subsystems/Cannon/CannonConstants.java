package frc.robot.subsystems.Cannon;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CannonConstants {
    //id
        public static final int MOTOR_ID = 0;
        public static final int SENSOR_ID = 0;


        //peaks
        public static final double PEAK_FORWARD_VOLTAGE = 11.5;
        public static final double PEAK_REVERSE_VOLTAGE = 11.5;
        public static final double CURRENT_PEAK = 60;
       
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;


        //other
        public static final double MOTOR_SPEED = 0.7;
        public static final double ADJUST_SPEED = 0.3;
        public static final double LOOSEN_TIME = 0.3;
}
