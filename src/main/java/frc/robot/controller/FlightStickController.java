package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.ControllerConstants;

/** WARNING: Not fully implemented */
public class FlightStickController implements Controller {

  CommandGenericHID controller = new CommandGenericHID(0);

  @Override
  public Trigger intake() {
    return controller.button(1);
  }

  @Override
  public Trigger scoreSpeaker() {
    return controller.povUp();
  }

  @Override
  public Trigger scoreAmp() {
    return controller.povDown();
  }

  @Override
  public Trigger failPosEstimate() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'failPosEstimate'. Why are you using the flight stick?!");
  }

  @Override
  public Trigger failNoteCam() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException(
        "Unimplemented method 'failNoteCam'. Why are you using the flight stick?!");
  }

  @Override
  public double getDriveX() {
    return MathUtil.applyDeadband(-controller.getRawAxis(1), ControllerConstants.DRIVE_DEADBAND);
  }

  @Override
  public double getDriveY() {
    return MathUtil.applyDeadband(-controller.getRawAxis(0), ControllerConstants.DRIVE_DEADBAND);
  }

  @Override
  public double getDriveTurn() {
    return MathUtil.applyDeadband(-controller.getRawAxis(2), 0.5);
  }

  public double getDriveBrake() {
    throw new UnsupportedOperationException(
        "Unimplemented method 'getDriveBrake'. Why are you using the flight stick?!");
  }

  @Override
  public Trigger adjustUp() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'adjustUp'");
  }

  @Override
  public Trigger adjustDown() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'adjustDown'");
  }

  @Override
  public Trigger scoreProtected() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'scoreProtected'");
  }

  @Override
  public Trigger alignAmp() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'alignAmp'");
  }

  @Override
  public Trigger climb() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'climb'");
  }
}
