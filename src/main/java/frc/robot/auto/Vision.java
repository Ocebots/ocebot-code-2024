package frc.robot.auto;

import frc.constants.VisionConstants;
import org.photonvision.PhotonCamera;

public class Vision {
  static {
    PhotonCamera.setVersionCheckEnabled(false);
  }

  private static final PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
  private static final PhotonCamera noteCamera = new PhotonCamera(VisionConstants.NOTE_CAMERA_NAME);

  public static PhotonCamera getCamera() {
    PhotonCamera.setVersionCheckEnabled(false);
    return camera;
  }

  public static PhotonCamera getNoteCamera() {
    PhotonCamera.setVersionCheckEnabled(false);
    return noteCamera;
  }
}
