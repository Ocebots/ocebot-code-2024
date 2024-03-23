package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
  Trigger intake();

  Trigger scoreSpeaker();

  Trigger scoreAmp();

  /**
   * Cause the position estimate to enter a failure state and default to base of the speaker for
   * shooting purposes
   */
  Trigger failPosEstimate();

  /**
   * Cause the intake to no longer rely on cameras to auto align but instead allow the driver to
   * take manual control
   */
  Trigger failNoteCam();

  /** Positive x is away */
  double getDriveX();

  /** Positive y is to the left */
  double getDriveY();

  /** CCW is positive */
  double getDriveTurn();

  /** From 0 to 1, positive is brake */
  double getDriveBrake();

  // TODO: Climb

  Trigger adjustUp();

  Trigger adjustDown();

  Trigger scoreProtected();

  Trigger alignAmp();

  Trigger climb();
}
