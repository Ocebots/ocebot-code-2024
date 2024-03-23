// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto.AutoAmpAlign;
import frc.robot.auto.AutoIntake;
import frc.robot.auto.Autos;
import frc.robot.controller.Controller;
import frc.robot.controller.XboxSoloController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class RobotContainer {
  DriveSubsystem drive = new DriveSubsystem();
  ShootSubsystem shooter = new ShootSubsystem();
  IntakeSubsystem intake = new IntakeSubsystem();
  Controller controller = new XboxSoloController();
  SendableChooser<Command> autos;

  public RobotContainer() {
    configureBindings();

    autos = new SendableChooser<>();

    autos.addOption("4 Piece Close", Autos.fourPieceClose(drive, intake, shooter));
    autos.addOption("3 Piece Far", Autos.threePieceFar(drive, intake, shooter));
    autos.addOption("2 Piece Out the Way", Autos.twoPieceOutOfTheWay(drive, intake, shooter));

    NamedCommands.registerCommand("Intake", new AutoIntake(drive, intake, shooter));
    NamedCommands.registerCommand("Shoot", shooter.scoreSpeaker(drive));

    SmartDashboard.putData("Select Auto", autos);

    drive.setDefaultCommand(
        Commands.run(() -> drive.defaultPeriodic(controller), drive)); // Maybe change this?
  }

  public void periodic() {
    drive.logData();
  }

  private void configureBindings() {
    this.controller.intake().onTrue(intake.intake(shooter));

    this.controller.scoreProtected().onTrue(shooter.scoreProtected());

    this.controller.adjustUp().whileTrue(shooter.adjust(true, intake));
    this.controller.adjustDown().whileTrue(shooter.adjust(false, intake));

    this.controller.scoreSpeaker().onTrue(shooter.scoreSpeaker(drive));
    this.controller.scoreAmp().onTrue(shooter.scoreAmp());

    this.controller.alignAmp().whileTrue(new AutoAmpAlign(drive, shooter));
    this.controller.failPosEstimate().whileTrue(new AutoIntake(drive, intake, shooter));

    this.controller.climb().whileTrue(shooter.climb());
  }

  public Command getAutonomousCommand() {
    return autos.getSelected();
  }
}
