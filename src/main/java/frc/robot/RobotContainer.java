// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoAmpAlign;
import frc.robot.controller.Controller;
import frc.robot.controller.XboxSoloController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class RobotContainer {
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShootSubsystem shooter = new ShootSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  Controller controller = new XboxSoloController();

  public RobotContainer() {
    configureBindings();

    driveSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              driveSubsystem.drive(
                  controller.getDriveX(),
                  controller.getDriveY(),
                  controller.getDriveTurn(),
                  true,
                  true);
            },
            driveSubsystem)); // Maybe change this?
  }

  public void periodic() {
    driveSubsystem.logData();
  }

  private void configureBindings() {
    this.controller.intake().onTrue(intake.intake(shooter));

    this.controller.scoreProtected().onTrue(shooter.scoreProtected());

    this.controller.adjustUp().whileTrue(shooter.adjust(true));
    this.controller.adjustDown().whileTrue(shooter.adjust(false));

    this.controller.scoreSpeaker().onTrue(shooter.scoreSpeaker(driveSubsystem));
    this.controller.scoreAmp().onTrue(shooter.scoreAmp());

    this.controller.alignAmp().whileTrue(new AutoAmpAlign(driveSubsystem, shooter));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
