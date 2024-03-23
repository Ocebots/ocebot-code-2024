package frc.robot.auto;

import frc.constants.VisionConstants;
import org.photonvision.PhotonCamera;

public class Vision {
  private static final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private static final PhotonCamera noteCamera = new PhotonCamera(VisionConstants.NOTE_CAMERA_NAME);

  public static PhotonCamera getCamera() {
    return camera;
  }

  public static PhotonCamera getNoteCamera() {
    return noteCamera;
  }
}
