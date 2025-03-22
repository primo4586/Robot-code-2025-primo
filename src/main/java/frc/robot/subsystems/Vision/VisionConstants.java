package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants { // Todo: find values
        //! DONT FORGET these values are in METERS not C"M!!
    // right camera
    public static final String RIGHT_CAMERA_NAME = "rightCamera";
    public static final Transform3d RIGHT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.265, -0.07, 0.6275),
            new Rotation3d(0, 0, 0));

    // left camera
    public static final String LEFT_CAMERA_NAME = "leftCamera";
    public static final Transform3d LEFT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.04, -0.26, 0.17),
            new Rotation3d(0, 0, 0));

    // front camera
    public static final String FRONT_CAMERA_NAME = "frontCamera";
    /*
     * in ISR2 the wanted position most likely wasn't true and the value of y need to be 0.1 to work on 
     * only one of the sides of the reefs. 
     */
    //TODO push back the reefCamera it and maybe add another camera. 
//     public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.215,-0.045, 0.295),
//             new Rotation3d(0, 0, 0));
    public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0,0,0),
            new Rotation3d(0, 0, 0));

        // reef camera
    public static final String REEF_CAMERA_NAME = "reefCamera";
    public static final Transform3d REEF_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, -0, 0),
            new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeWelded);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(3, 3, 0.5); // TODO: tune to
                                                                                                       // each camera
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);

    public static final Pose2d leftReefTargetGoal = new Pose2d(0.38, 0.01, new Rotation2d()); // TODO: tune

    public static final Pose2d rightReefTargetGoal = new Pose2d(0.44, 0.06 , new Rotation2d(Math.toRadians(-5)));

}
