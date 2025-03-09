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

    public static final Pose2d BLUE_REEF_A_POSITION = new Pose2d(3.43,4.12,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_B_POSITION = new Pose2d(3.43,3.760,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_C_POSITION = new Pose2d(3.88,3.17,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_D_POSITION = new Pose2d(4.24,3.02,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_E_POSITION = new Pose2d(4.94,3.07,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_F_POSITION = new Pose2d(5.25,3.29,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_G_POSITION = new Pose2d(5.52,3.94,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_H_POSITION = new Pose2d(5.52,4.28,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_I_POSITION = new Pose2d(5.09, 4.92,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_J_POSITION = new Pose2d(4.8,5.04,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_K_POSITION = new Pose2d(3.881,5.185,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d BLUE_REEF_L_POSITION = new Pose2d(5.200,5.100,new Rotation2d(Math.toRadians(-60)));

    public static final Pose2d RED_REEF_A_POSITION = new Pose2d(14.10,3.90,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_B_POSITION = new Pose2d(14.12,4.31,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_C_POSITION = new Pose2d(13.69,4.88,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_D_POSITION = new Pose2d(13.31,5.08,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_E_POSITION = new Pose2d(12.61,5.03,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_F_POSITION = new Pose2d(12.31,4.79,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_G_POSITION = new Pose2d(11.98,4.15,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_H_POSITION = new Pose2d(11.97,3.77,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_I_POSITION = new Pose2d(12.42,3.15,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_J_POSITION = new Pose2d(12.75,2.98,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_K_POSITION = new Pose2d(13.80,3.22,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d RED_REEF_L_POSITION = new Pose2d(13.48,3.05,new Rotation2d(Math.toRadians(120)));
}
