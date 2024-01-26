package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.ControllerConstants;

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
    return MathUtil.applyDeadband(-controller.getLeftY(), ControllerConstants.DRIVE_DEADBAND);
  }

  @Override
  public double getDriveY() {
    return MathUtil.applyDeadband(-controller.getLeftX(), ControllerConstants.DRIVE_DEADBAND);
  }

  @Override
  public double getDriveTurn() { // TODO: Do we need the negative?
    return MathUtil.applyDeadband(-controller.getRightX(), ControllerConstants.DRIVE_DEADBAND);
  }
}
