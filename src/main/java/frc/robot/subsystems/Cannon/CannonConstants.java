package frc.robot.subsystems.Cannon;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CannonConstants {
    //id
        public static final int MOTOR_ID = 23;
        public static final int SENSOR_ID = 8;


        //peaks
        public static final double PEAK_FORWARD_VOLTAGE = 11.5;
        public static final double PEAK_REVERSE_VOLTAGE = 11.5;
        public static final int CURRENT_PEAK = 60;
       
        public static final IdleMode NEUTRAL_MODE = IdleMode.kCoast;
        public static final Boolean INVERTED = true;


        //other
        public static final double MOTOR_SPEED = 0.7;
        public static final double ADJUST_SPEED = 0.3;
        public static final double LOOSEN_TIME = 1.5;
        public static final double CATCH_SPEED = 0.2;
        public static final double CATCH_TIME = 0.1;
}
