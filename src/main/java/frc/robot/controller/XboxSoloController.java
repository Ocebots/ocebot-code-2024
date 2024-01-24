package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxSoloController implements Controller {
  CommandXboxController controller = new CommandXboxController(0);

  @Override
  public Trigger intake() {
    return controller.y();
  }

  @Override
  public Trigger scoreSpeaker() {
    return controller.leftBumper();
  }

  @Override
  public Trigger scoreAmp() {
    return controller.rightBumper();
  }

  @Override
  public Trigger failPosEstimate() {
    return controller.b();
  }

  @Override
  public Trigger failNoteCam() {
    return controller.x();
  }

  @Override
  public double getDriveX() {
    return -controller.getLeftY();
  }

  @Override
  public double getDriveY() {
    return -controller.getLeftX();
  }

  @Override
  public double getDriveTurn() {
    return controller.getRightX();
  }
}
