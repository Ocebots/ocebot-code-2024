// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.constants.ControllerConstants;
import frc.robot.subsystems.ShootSubsystem;

public class RobotContainer {
  // DriveSubsystem driveSubsystem = new DriveSubsystem();
  ShootSubsystem shooter = new ShootSubsystem();
  CommandXboxController controller =
      new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

  double angle = 0.0;
  double height = 0.0;

  public RobotContainer() {
    configureBindings();
    // driveSubsystem.register();
    //
    // driveSubsystem.setDefaultCommand(
    //     Commands.run(
    //         () -> {
    //           driveSubsystem.drive(
    //               -MathUtil.applyDeadband(
    //                   controller.getLeftY(), ControllerConstants.DRIVE_DEADBAND),
    //               -MathUtil.applyDeadband(
    //                   controller.getLeftX(), ControllerConstants.DRIVE_DEADBAND),
    //               -MathUtil.applyDeadband(
    //                   controller.getRightX(), ControllerConstants.DRIVE_DEADBAND),
    //               true,
    //               true);
    //         },
    //         driveSubsystem)); // Maybe change this?

    angle = shooter.getAngleRads();
  }

  private void configureBindings() {
    this.controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.angle += 5 * Math.PI / 180;
                  shooter.setAngleRaw(Rotation2d.fromRadians(angle));
                }));
    this.controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.angle -= 5 * Math.PI / 180;
                  shooter.setAngleRaw(Rotation2d.fromRadians(angle));
                }));

    this.controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.height += 0.02;
                  shooter.setHeightRaw(height);
                }));
    this.controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  this.height -= 0.02;
                  shooter.setHeightRaw(height);
                }));

    // this.controller
    //     .rightBumper()
    //     .whileTrue(Commands.run(() -> this.driveSubsystem.setX(), this.driveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
