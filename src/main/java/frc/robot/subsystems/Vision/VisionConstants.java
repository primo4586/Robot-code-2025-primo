package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants { // Todo: find values
    //right camera
    public static final String RIGHT_CAMERA_NAME = "rightCamera";
    public static final Transform3d RIGHT_CAMERA_TO_ROBOT =  new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    //left camera
    public static final String LEFT_CAMERA_NAME = "leftCamera";
    public static final Transform3d LEFT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    //front camera
    public static final String FRONT_CAMERA_NAME = "leftCamera";
    public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
    AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8); //TODO: tune to each camera
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    
}
