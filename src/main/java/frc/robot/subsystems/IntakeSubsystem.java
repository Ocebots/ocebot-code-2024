package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private Command runIntake() {
    return runIntake(false);
  }

  public Command intake(ShootSubsystem shooter) {
    return intake(shooter, false);
  }

  public Command intake(ShootSubsystem shooter, boolean reversed) {
    return shooter
        .intakeMode()
        .andThen(runIntake(reversed).raceWith(shooter.waitForIntake()))
        .andThen(shooter.completeIntake());
  }
}
