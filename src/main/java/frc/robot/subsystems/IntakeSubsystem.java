package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.constants.CANMappings;
import frc.constants.IntakeConstants;

public class IntakeSubsystem {
  private CANSparkMax motor = new CANSparkMax(CANMappings.INTAKE, MotorType.kBrushless);

  public IntakeSubsystem() {}

  /**
   * @return a command that will run forever that sets the motor to intake
   */
  private Command runIntake() {
    return Commands.runEnd(() -> motor.set(IntakeConstants.INTAKE_SPEED), () -> motor.set(0));
  }

  public Command intake(ShootSubsystem shooter) {
    return null; // TODO: Intake a note fully, so that it is ready to be shot
  }
}
