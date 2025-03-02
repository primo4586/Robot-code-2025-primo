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
    // right camera
    public static final String RIGHT_CAMERA_NAME = "rightCamera";
    public static final Transform3d RIGHT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));

    // left camera
    public static final String LEFT_CAMERA_NAME = "leftCamera";
    public static final Transform3d LEFT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));

    // front camera
    public static final String FRONT_CAMERA_NAME = "frontCamera";
    /*
     * in ISR2 the wanted position most likely wasn't true and the value of y need to be 0.1 to work on 
     * only one of the sides of the reefs. 
     */
    //todo push back the camera alighn it and maybe add another camera. 
    public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0.27,-0.5, 0.34),
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
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(3, 3, Integer.MAX_VALUE); // TODO: tune to
                                                                                                       // each camera
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(5, 5, Integer.MAX_VALUE);

    public static final Pose2d leftReefTargetGoal = new Pose2d(0.319, -0.085, new Rotation2d(14.93));

    public static final Pose2d rightReefTargetGoal = new Pose2d(0, 0, new Rotation2d(1));

}
