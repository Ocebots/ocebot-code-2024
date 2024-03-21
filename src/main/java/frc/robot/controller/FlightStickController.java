package frc.robot.controller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.ControllerConstants;

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
    throw new UnsupportedOperationException("Unimplemented method 'failPosEstimate'");
  }

  @Override
  public Trigger failNoteCam() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'failNoteCam'");
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
}
