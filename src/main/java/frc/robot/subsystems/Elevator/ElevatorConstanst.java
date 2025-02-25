package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class ElevatorConstanst { // todo: tune values 
    public static final int MASTER_TALONFX_ID = 30;
    public static final int FOLLOW_TALONFX_ID = 31;

    public static final double ROLLER_RADIUS = 0.048; // In meters
    public static final double MAXIMUM_HIGHT = 2.516845703125; // In meters
    public static final double GEAR_RATIO = 15 ;  // todo: i'm not sure if it true 

    public static TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration();
    static {

      ELEVATOR_CONFIG.CurrentLimits.SupplyCurrentLimitEnable = true;
      ELEVATOR_CONFIG.CurrentLimits.StatorCurrentLimit = 60; //^ i'm not sure if it true
      ELEVATOR_CONFIG.Voltage.PeakForwardVoltage = 11.5;
      ELEVATOR_CONFIG.Voltage.PeakReverseVoltage = -11.5;


      ELEVATOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      ELEVATOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAXIMUM_HIGHT; 
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      ELEVATOR_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -2;

      ELEVATOR_CONFIG.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      ELEVATOR_CONFIG.Feedback.SensorToMechanismRatio = GEAR_RATIO / (ROLLER_RADIUS * 2 * Math.PI);

      ELEVATOR_CONFIG.Slot0.kG = 0.3; // Volts to overcome gravity
      ELEVATOR_CONFIG.Slot0.kS = 1.4258; // Volts to overcome static friction
      ELEVATOR_CONFIG.Slot0.kV = 3.0932; // Volts for a velocity target of 1 rps
      ELEVATOR_CONFIG.Slot0.kA = 0.75; // Volts for an acceleration of 1 rps/s 9.0127
      ELEVATOR_CONFIG.Slot0.kP = 40;
      ELEVATOR_CONFIG.Slot0.kI = 0.0;
      ELEVATOR_CONFIG.Slot0.kD = 10;

      ELEVATOR_CONFIG.MotionMagic.MotionMagicCruiseVelocity = 10;
      ELEVATOR_CONFIG.MotionMagic.MotionMagicAcceleration = 3 ;      
    }

    public static final double HIGHT_OF_THE_GROUND = 0;
    public static final double MINIMUN_POSITION_ERROR = 0.05;
    public static final double MINIMUN_VELOCITY_ERROR = 0.05;

    public static final double MOVE_POWER = 0.3;

    public static final double L1_HEIGHT = 0;
    public static final double L2_HEIGHT = 0.94384765625 -0.24;
    public static final double L3_HEIGHT = 1.534423828125 -0.24;
    public static final double L4_HEIGHT = 2.553173828125 - 0.14; // todo tune values
    //our reef height: 2.553173828125 - 0.10;
    //real reef height: 2.31689453125; 

    public static final double LOW_ALGEA_HEIGHT = 0.89;
    public static final double HIGH_ALGEA_HEIGHT = 0; //TODO: FIND VALUE

}
