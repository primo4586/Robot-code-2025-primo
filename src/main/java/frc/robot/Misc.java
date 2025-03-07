package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Misc {
    public static final String CANIVOR_NAME = "canBus";

    //swerve Commands
    //^the Rotation2d is false bit it is uslless
    public static final Pose2d BLUE_REEF_CENTER_POSITION = new Pose2d (4.5,4,new Rotation2d()); 
    public static final Pose2d RED_REEF_CENTER_POSITION = new Pose2d (13,4,new Rotation2d()); 

    public static final double REEF_1_ANGLE = 180; //^there is a good chance i am wrong
    public static final double REEF_2_ANGLE = 120;
    public static final double REEF_3_ANGLE = 60;
    public static final double REEF_4_ANGLE = 0;
    public static final double REEF_5_ANGLE = 300;
    public static final double REEF_6_ANGLE = 240;

    public static final Pose2d BLUE_REEF_A_POSITION = new Pose2d(3.171,4.100,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_B_POSITION = new Pose2d(3.171,3.750,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_C_POSITION = new Pose2d(1.175,7.022,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_D_POSITION = new Pose2d(4.064,2.742,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_E_POSITION = new Pose2d(5.109,2.884,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_F_POSITION = new Pose2d(5.100,3.010,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_G_POSITION = new Pose2d(5.095,2.850,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_H_POSITION = new Pose2d(1.175,7.022,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_I_POSITION = new Pose2d(5.226,5.136,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_J_POSITION = new Pose2d(4.915,5.284,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_K_POSITION = new Pose2d(3.881,5.185,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d BLUE_REEF_L_POSITION = new Pose2d(5.200,5.100,new Rotation2d(Math.toRadians(-60)));

    public static final Pose2d RED_REEF_A_POSITION = new Pose2d(14.362,3.976,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_B_POSITION = new Pose2d(14.362,4.288,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_C_POSITION = new Pose2d(13.767,5.127,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_D_POSITION = new Pose2d(13.494,5.283,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_E_POSITION = new Pose2d(12.451,5.165,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_F_POSITION = new Pose2d(12.158,4.990,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_G_POSITION = new Pose2d(11.720,4.083,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_H_POSITION = new Pose2d(11.720,3.752,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_I_POSITION = new Pose2d(12.373,2.913,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_J_POSITION = new Pose2d(12.665,2.767,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_K_POSITION = new Pose2d(13.679,2.865,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d RED_REEF_L_POSITION = new Pose2d(13.923,3.050,new Rotation2d(Math.toRadians(120)));
}
