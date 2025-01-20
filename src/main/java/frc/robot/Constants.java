package frc.robot;


import java.util.Dictionary;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class Constants {
   




    public class CannonConstants{ // TODO: find values
        //id
        public static final int MOTOR_ID = 0;
        public static final int SENSOR_ID = 0;


        //configs
        public static final double PEAK_FORWARD_VOLTAGE = 0;
        public static final double PEAK_REVERSE_VOLTAGE = 0;
       
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;


        //other
        public static final double MOTOR_SPEED = 0;
        public static final double ADJUST_SPEED = 0;
        public static final double LOOSEN_TIME = 0.3;
    }


    public class RobotConstants{
        public static final String CANIVORE_NAME = "canBus";
    }
}


