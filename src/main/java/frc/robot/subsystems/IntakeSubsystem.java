package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.CANMappings;
import frc.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax motor = new CANSparkMax(CANMappings.INTAKE, MotorType.kBrushless);

  public IntakeSubsystem() {
    motor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
    motor.setIdleMode(IntakeConstants.IDLE_MODE);
  }

  /**
   * @return a command that will run forever that sets the motor to intake
   */
  private Command runIntake(boolean inverted) {
    return Commands.runEnd(
        () -> motor.set(IntakeConstants.SPEED * (inverted ? -1 : 1)), () -> motor.set(0), this);
  }

  public Command intake(ShootSubsystem shooter) {
    return shooter
        .intakeMode()
        .andThen(runIntake(false).raceWith(shooter.waitForIntake()).withTimeout(IntakeConstants.INTAKE_TIMEOUT))
        .andThen(new ConditionalCommand(Commands.run(shooter::completeIntake), ejectNote(shooter), () -> shooter.intakeSuccess)).finallyDo(() -> shooter.intakeSuccess = false);
  }

  public Command ejectNote(ShootSubsystem shooter){
    return runIntake(true).withTimeout(IntakeConstants.EJECT_DURATION);
  }
}
