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

    public static final Pose2d BLUE_REEF_A_POSITION = new Pose2d(3.45,4.12,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_B_POSITION = new Pose2d(3.45,3.760,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d BLUE_REEF_C_POSITION = new Pose2d(3.88,3.17,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_D_POSITION = new Pose2d(4.24,3.02,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d BLUE_REEF_E_POSITION = new Pose2d(4.94,3.07,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_F_POSITION = new Pose2d(5.25,3.29,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d BLUE_REEF_G_POSITION = new Pose2d(5.50,3.94,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_H_POSITION = new Pose2d(5.50,4.28,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d BLUE_REEF_I_POSITION = new Pose2d(5.13, 4.91,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_J_POSITION = new Pose2d(4.76,5.07,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d BLUE_REEF_K_POSITION = new Pose2d(4.06,4.97,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d BLUE_REEF_L_POSITION = new Pose2d(3.71,4.84,new Rotation2d(Math.toRadians(-60)));

    public static final Pose2d RED_REEF_A_POSITION = new Pose2d(14.10,3.90,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_B_POSITION = new Pose2d(14.10,4.27,new Rotation2d(Math.toRadians(180)));
    public static final Pose2d RED_REEF_C_POSITION = new Pose2d(13.69,4.88,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_D_POSITION = new Pose2d(13.36,5.04,new Rotation2d(Math.toRadians(-120)));
    public static final Pose2d RED_REEF_E_POSITION = new Pose2d(12.61,5.03,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_F_POSITION = new Pose2d(12.31,4.79,new Rotation2d(Math.toRadians(-60)));
    public static final Pose2d RED_REEF_G_POSITION = new Pose2d(12.03,4.12,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_H_POSITION = new Pose2d(12.03,3.77,new Rotation2d(Math.toRadians(0)));
    public static final Pose2d RED_REEF_I_POSITION = new Pose2d(12.46,3.22,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_J_POSITION = new Pose2d(12.75,2.98,new Rotation2d(Math.toRadians(60)));
    public static final Pose2d RED_REEF_K_POSITION = new Pose2d(13.51,3.06,new Rotation2d(Math.toRadians(120)));
    public static final Pose2d RED_REEF_L_POSITION = new Pose2d(13.80,3.28,new Rotation2d(Math.toRadians(120)));
}
