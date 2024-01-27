// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controller.Controller;
import frc.robot.controller.XboxSoloController;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  DriveSubsystem driveSubsystem = new DriveSubsystem();

  Controller controller = new XboxSoloController();

  public RobotContainer() {
    configureBindings();
    driveSubsystem.register();

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

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
