package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.ControllerConstants;

/*
 Intake - A Button
   Score Speaker - Left Trigger
   Score Amp - Right Bumper
   Position Estimate Fallback - B Button
   Note Cam Fallback - X Button
   Drive - Left Stick
   Turn - Right Stick
*/
public class XboxSoloController implements Controller {
  CommandXboxController controller = new CommandXboxController(0);

  @Override
  public Trigger intake() {
    return controller.b();
  }

  @Override
  public Trigger manualIntake() {
    return controller.a();
  }

  @Override
  public Trigger scoreSpeaker() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger scoreAmp() {
    return controller.rightTrigger();
  }

  @Override
  public double getDriveX() {
    double applyDeadband =
        MathUtil.applyDeadband(-controller.getLeftY(), ControllerConstants.DRIVE_DEADBAND);
    return Math.pow(applyDeadband, 2) * Math.signum(applyDeadband);
  }

  @Override
  public double getDriveY() {
    double applyDeadband =
        MathUtil.applyDeadband(-controller.getLeftX(), ControllerConstants.DRIVE_DEADBAND);
    return Math.pow(applyDeadband, 2) * Math.signum(applyDeadband);
  }

  @Override
  public double getDriveTurn() {
    return MathUtil.applyDeadband(-controller.getRightX(), ControllerConstants.DRIVE_DEADBAND);
  }

  @Override
  public Trigger adjustUp() {
    return controller.povUp();
  }

  @Override
  public Trigger stuck() {
    return controller.povRight();
  }

  @Override
  public Trigger adjustDown() {
    return controller.povDown();
  }

  @Override
  public Trigger scoreHigh() {
    return controller.leftBumper();
  }

  @Override
  public Trigger climb() {
    return controller.y();
  }
}
