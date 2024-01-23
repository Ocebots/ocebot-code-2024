package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.CANMappings;

public class IntakeSubsystem {
  private CANSparkMax motor = new CANSparkMax(CANMappings.INTAKE, MotorType.kBrushless);

  public IntakeSubsystem() {}

  private Command runIntake() {
    return null; // TODO: Run the intake motors, nothing else
  }

  public Command intake(ShootSubsystem shooter) {
    return null; // TODO: Intake a note fully, so that it is ready to be shot
  }
}
