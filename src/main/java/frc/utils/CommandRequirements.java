package frc.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashSet;

public class CommandRequirements extends Command {
  private Command command;

  private CommandRequirements(Command command) {
    this.command = command;
  }

  public static Command withRequirements(Command command, Subsystem... subsystems) {
    CommandRequirements commandRequirements = new CommandRequirements(command);

    HashSet<Subsystem> subsystemsSet = new HashSet<>();

    for (Subsystem subsystem : subsystems) {
      subsystemsSet.add(subsystem);
    }

    commandRequirements.m_requirements = subsystemsSet;

    return commandRequirements;
  }

  @Override
  public void initialize() {
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean value) {
    command.end(value);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
}
