package frc.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {
  public static final PhotonCamera CAMERA = new PhotonCamera("apriltag");
  public static final AprilTagFieldLayout LAYOUT =
      AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  public static final PoseStrategy STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  public static final Transform3d CAMERA_POSITION =
      new Transform3d(0.3048, 0.2159, 0.3556, new Rotation3d(0, -0.523599, 0));
}
