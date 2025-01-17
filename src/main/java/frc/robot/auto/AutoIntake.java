package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import java.util.HashSet;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AutoIntake extends SequentialCommandGroup {
  PhotonTrackedTarget target;
  ChassisSpeeds driveBack = null;

  public AutoIntake(
      DriveSubsystem driveSubsystem,
      IntakeSubsystem intakeSubsystem,
      ShootSubsystem shootSubsystem) {
    PhotonCamera camera = Vision.getNoteCamera();

    HashSet<Subsystem> subsystems = new HashSet<>();

    subsystems.add(driveSubsystem);
    subsystems.add(intakeSubsystem);
    subsystems.add(shootSubsystem);

    addCommands(
        Commands.waitUntil(() -> camera.getLatestResult().hasTargets()),
        Commands.defer(
            () -> {
              PIDController controller = new PIDController(0.01, 0, 0);
              controller.setTolerance(3);

              return new PIDCommand(
                      controller,
                      () -> {
                        if (camera.getLatestResult().hasTargets()) {
                          target = camera.getLatestResult().getBestTarget();
                        }

                        return target.getYaw();
                      },
                      () -> 0,
                      (val) ->
                          driveSubsystem.drive(
                              0, 0, Math.max(Math.min(val, 0.5), -0.5), true, false),
                      driveSubsystem)
                  .until(() -> controller.atSetpoint())
                  .andThen(
                      intakeSubsystem
                          .intake(shootSubsystem)
                          .deadlineWith(
                              Commands.runOnce(
                                      () -> {
                                        driveBack =
                                            ChassisSpeeds.fromRobotRelativeSpeeds(
                                                -0.2,
                                                0.0,
                                                0.0,
                                                driveSubsystem
                                                    .getPose()
                                                    .getRotation()
                                                    .plus(
                                                        DriverStation.getAlliance()
                                                                    .orElse(Alliance.Blue)
                                                                == Alliance.Blue
                                                            ? Rotation2d.fromDegrees(0)
                                                            : Rotation2d.fromDegrees(180)));
                                      })
                                  .andThen(
                                      Commands.runEnd(
                                          () ->
                                              driveSubsystem.drive(
                                                  driveBack.vxMetersPerSecond,
                                                  driveBack.vyMetersPerSecond,
                                                  driveBack.omegaRadiansPerSecond,
                                                  true,
                                                  false),
                                          () -> driveSubsystem.drive(0, 0, 0, false, false),
                                          driveSubsystem))
                                  .raceWith(
                                      Commands.waitUntil(
                                              () -> !camera.getLatestResult().hasTargets())
                                          .andThen(Commands.waitSeconds(0.6)))));
            },
            subsystems));
  }
}
