package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.constants.ShooterConstants.FlywheelConstants;

public class FlywheelSubsystem extends PIDSubsystem {
  public final CANSparkFlex motor;
  public final RelativeEncoder encoder;

  private final SimpleMotorFeedforward feedforward;

  public FlywheelSubsystem(
      int canId, boolean inverted, double pGain, double sGain, double vGain, double aGain) {
    super(new PIDController(pGain, 0, 0));

    feedforward = new SimpleMotorFeedforward(sGain, vGain, aGain);

    m_controller.setTolerance(FlywheelConstants.TOLERANCE);

    this.motor = new CANSparkFlex(canId, MotorType.kBrushless);
    this.encoder = motor.getEncoder();

    this.motor.restoreFactoryDefaults();

    this.motor.setSmartCurrentLimit(FlywheelConstants.CURRENT_LIMIT);
    this.motor.setIdleMode(FlywheelConstants.IDLE_MODE);

    this.encoder.setPositionConversionFactor(FlywheelConstants.POSITION_CONVERSION_FACTOR);
    this.encoder.setVelocityConversionFactor(FlywheelConstants.VELOCITY_CONVERSION_FACTOR);

    this.motor.setInverted(inverted);

    this.motor.burnFlash();
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("shooter/velocity/" + getName(), encoder.getVelocity());
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    motor.setVoltage(output + feedforward.calculate(setpoint));
  }

  @Override
  protected double getMeasurement() {
    return encoder.getVelocity();
  }

  public Command setVelocity(double velocity) {
    return Commands.runOnce(
            () -> {
              enable();
              setSetpoint(velocity);
            },
            this)
        .andThen(Commands.run(() -> {}, this))
        .finallyDo(this::disable);
  }

  public Command waitForVelocity(double velocity) {
    return Commands.waitUntil(() -> m_controller.atSetpoint()).withTimeout(1.5);
  }
}
