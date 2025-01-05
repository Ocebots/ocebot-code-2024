package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Controller {
  Trigger intake();
  Trigger manualIntake();

  Trigger scoreSpeaker();

  Trigger scoreAmp();

  /** Positive x is away */
  double getDriveX();

  /** Positive y is to the left */
  double getDriveY();

  /** CCW is positive */
  double getDriveTurn();

  Trigger adjustUp();

  Trigger adjustDown();

  Trigger stuck();

  Trigger scoreHigh();

  Trigger climb();
}
