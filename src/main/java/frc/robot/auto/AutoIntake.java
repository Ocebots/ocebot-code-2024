package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.AutoConstants;
import frc.constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import java.util.List;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoIntake extends SequentialCommandGroup {
  public AutoIntake(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShootSubsystem shootSubsystem) {
    addCommands(
        Commands.waitUntil(() -> Vision.getNoteCamera().getLatestResult().hasTargets()),
        Commands.deferredProxy(
            () -> {
              PhotonTrackedTarget target = Vision.getNoteCamera().getLatestResult().getBestTarget();
              double yaw = target.getYaw();
              double pitch = target.getPitch();

              double range =
                  PhotonUtils.calculateDistanceToTargetMeters(
                      VisionConstants.CAMERA_HEIGHT_METERS,
                      VisionConstants.NOTE_TARGET_HEIGHT_METERS,
                      VisionConstants.CAMERA_PITCH_RADIANS,
                      Units.degreesToRadians(pitch));

              Translation2d targetTranslation =
                  PhotonUtils.estimateCameraToTargetTranslation(range, Rotation2d.fromDegrees(-yaw))
                      .rotateBy(Rotation2d.fromDegrees(180))
                      .rotateBy(
                          driveSubsystem
                              .getPose()
                              .getRotation()); // TODO: Is this the correct rotation?

              Pose2d targetPose =
                  driveSubsystem
                      .getPose()
                      .transformBy(new Transform2d(targetTranslation, Rotation2d.fromRadians(0)));

              List<Translation2d> points =
                  PathPlannerPath.bezierFromPoses(driveSubsystem.getPose(), targetPose);

              PathPlannerPath path =
                  new PathPlannerPath(
                      points,
                      AutoConstants.PATH_CONSTRAINTS,
                      new GoalEndState(
                          0, Rotation2d.fromDegrees(0))); // TODO: Ensure final rotation is correct

              path.preventFlipping = true;

              // TODO: Keep updating the target estimate

              return AutoBuilder.followPath(path).alongWith(intakeSubsystem.intake(shootSubsystem));
            }));
  }
}
