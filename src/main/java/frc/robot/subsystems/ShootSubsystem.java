package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSubsystem {
  public ShootSubsystem() {}

  /*
   * @param velocity meters per second of the ring
   */
  private Command shoot(double velocity) {
    return null; // TODO: Make it shoot at the current angle
  }

  private Command setAngle(Rotation2d newAngle) {
    return null; // TODO: Set the angle of the arm to a certain amount
  }

  private Command setHeight(double meters) {
    return null; // TODO: Set the height of the arm to a certain amount
  }

  public Command underStageMode() {
    return null; // TODO: Set the angle and height to be able to go under the stage
  }

  public Command scoreAmp() {
    return null; // TODO: Set the angle and height to go under the stage
  }

  public Command scoreSpeaker(Pose2d currentPos) {
    return null; // TODO: Using the current posistion of the robot, score in speaker. If unable,
    // do nothing
  }

  public Command climb() {
    return null; // TODO: Climb onto the chain
  }
}
