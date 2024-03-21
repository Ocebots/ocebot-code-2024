package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import java.util.HashSet;
import java.util.List;

public class AutoAmpAlign extends DeferredCommand {
  public AutoAmpAlign(DriveSubsystem driveSubsystem, ShootSubsystem shooter) {
    super(() -> create(driveSubsystem, shooter), requirements(driveSubsystem, shooter));
  }

  private static HashSet<Subsystem> requirements(
      DriveSubsystem driveSubsystem, ShootSubsystem shooter) {
    HashSet<Subsystem> requirements = new HashSet<>();
    requirements.add(driveSubsystem);
    requirements.add(shooter);
    return requirements;
  }

  private static Command create(DriveSubsystem driveSubsystem, ShootSubsystem shooter) {
    List<Translation2d> points =
        PathPlannerPath.bezierFromPoses(
            driveSubsystem.getPose(),
            new Pose2d(
                (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue)
                    ? 1.8415
                    : 14.7008,
                7.7642,
                Rotation2d.fromDegrees(90)));

    PathPlannerPath path =
        new PathPlannerPath(
            points,
            AutoConstants.PATH_CONSTRAINTS,
            new GoalEndState(0, Rotation2d.fromDegrees(90)));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path).andThen(shooter.scoreAmp());
  }
}
