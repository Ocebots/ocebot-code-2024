package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.constants.CANMappings;
import frc.constants.ShooterConstants;
import frc.constants.ShooterConstants.HeightConstants;

public class HeightSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax left = new CANSparkMax(CANMappings.ELEVATOR_LEFT, MotorType.kBrushless);
  private CANSparkMax right = new CANSparkMax(CANMappings.ELEVATOR_RIGHT, MotorType.kBrushless);

  private RelativeEncoder encoder = left.getEncoder();

  private ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          HeightConstants.STATIC_GAIN, HeightConstants.GRAVITY_GAIN, HeightConstants.VELOCITY_GAIN);

  public HeightSubsystem() {
    super(
        new ProfiledPIDController(
            HeightConstants.P_GAIN,
            HeightConstants.I_GAIN,
            HeightConstants.D_GAIN,
            HeightConstants.CONSTRAINTS),
        ShooterConstants.INTAKE_HEIGHT);

    m_controller.setTolerance(HeightConstants.TOLERANCE);

    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setSmartCurrentLimit(HeightConstants.CURRENT_LIMIT);
    right.setSmartCurrentLimit(HeightConstants.CURRENT_LIMIT);
    left.setIdleMode(HeightConstants.IDLE_MODE);
    right.setIdleMode(HeightConstants.IDLE_MODE);

    right.follow(left);

    encoder.setPositionConversionFactor(HeightConstants.POSITION_CONVERSION_FACTOR);
    encoder.setVelocityConversionFactor(HeightConstants.VELOCITY_CONVERSION_FACTOR);

    left.burnFlash();
    right.burnFlash();

    encoder.setPosition(0.0);

    enable();
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    left.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
  }

  @Override
  protected double getMeasurement() {
    return encoder.getPosition();
  }

  public Command setHeight(double height) {
    return Commands.runOnce(() -> this.setGoal(height))
        .andThen(
            Commands.waitUntil(m_controller::atGoal).deadlineWith(Commands.run(() -> {}, this)));
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("shooter/height/current", encoder.getPosition());
    SmartDashboard.putNumber("shooter/height/target", m_controller.getGoal().position);
  }
}
