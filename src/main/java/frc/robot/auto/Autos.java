package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class Autos {
  public static Command fourPieceClose(
      DriveSubsystem drive, IntakeSubsystem intake, ShootSubsystem shooter) {
    return Commands.sequence(
        shooter.scoreSpeaker(drive),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Left Subwoofer to Left Note")),
        new AutoIntake(drive, intake, shooter),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Left Note to Middle")),
        shooter.scoreSpeaker(drive),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Left Note to Middle Note")),
        new AutoIntake(drive, intake, shooter),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Middle Note to Middle")),
        shooter.scoreSpeaker(drive),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Middle Note to Right Note")),
        new AutoIntake(drive, intake, shooter),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Right Note to Middle")),
        shooter.scoreSpeaker(drive));
  }

  public static Command threePieceFar(
      DriveSubsystem drive, IntakeSubsystem intake, ShootSubsystem shooter) {
    return Commands.sequence(
        shooter.scoreSpeaker(drive),
        new AutoIntake(drive, intake, shooter),
        shooter.scoreSpeaker(drive),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Middle Note to Far")),
        new AutoIntake(drive, intake, shooter),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Far to Shoot")),
        shooter.scoreSpeaker(drive));
  }

  public static Command twoPieceOutOfTheWay(
      DriveSubsystem drive, IntakeSubsystem intake, ShootSubsystem shooter) {
    return Commands.sequence(
        shooter.scoreSpeaker(drive),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Out the Way to Far")),
        new AutoIntake(drive, intake, shooter),
        AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Far to Out the Way")),
        shooter.scoreSpeaker(drive));
  }

  public static Command twoPiece(
      DriveSubsystem drive, IntakeSubsystem intake, ShootSubsystem shooter) {
    return Commands.sequence(
        shooter.scoreSpeaker(drive),
        new AutoIntake(drive, intake, shooter),
        shooter.scoreSpeaker(drive));
  }
}
