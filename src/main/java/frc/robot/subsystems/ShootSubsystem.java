package frc.robot.subsystems;

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
}
