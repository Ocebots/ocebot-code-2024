// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  CommandXboxController controller =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
    driveSubsystem.register();

    driveSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              driveSubsystem.drive(
                  -MathUtil.applyDeadband(
                      controller.getLeftY(), ControllerConstants.DRIVE_DEADBAND),
                  -MathUtil.applyDeadband(
                      controller.getLeftX(), ControllerConstants.DRIVE_DEADBAND),
                  -MathUtil.applyDeadband(
                      controller.getRightX(), ControllerConstants.DRIVE_DEADBAND),
                  true,
                  true);
            },
            driveSubsystem)); // Maybe change this?
  }

  private void configureBindings() {
    this.controller.a().onTrue(Commands.runOnce(() -> this.driveSubsystem.zeroHeading(), this.driveSubsystem));
    this.controller
        .rightBumper()
        .whileTrue(Commands.run(() -> this.driveSubsystem.setX(), this.driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
