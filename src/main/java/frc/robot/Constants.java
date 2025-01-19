package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {
    public class RobotConstants{
        public static final String CANIVOR_NAME = "canBus";
    }
    public class GripperConstants{ //TODO: find values
        public static final int MOTOR_ID = 0;

        public static final double COLLECT_POWER = 0.5;
        public static final double HOLED_POWER = 0.15; 
        public static final double TOSS_POWER = -0.7;
        public static final double TOSS_TIME = 0.1;

        // settings
        public static final double VOLTAGE_LIMIT = 7;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

    }
    
}
