package frc.robot.subsystems.shooter;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.constants.CANMappings;
import frc.constants.ShooterConstants;
import frc.constants.ShooterConstants.TiltConstants;

public class TiltSubsystem extends ProfiledPIDSubsystem {
  private CANSparkMax left = new CANSparkMax(CANMappings.TILT_LEFT, MotorType.kBrushless);
  private CANSparkMax right = new CANSparkMax(CANMappings.TILT_RIGHT, MotorType.kBrushless);

  private AbsoluteEncoder encoder = left.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  private ArmFeedforward feedforward =
      new ArmFeedforward(
          TiltConstants.STATIC_GAIN, TiltConstants.GRAVITY_GAIN, TiltConstants.VELOCITY_GAIN);

  public TiltSubsystem() {
    super(
        new ProfiledPIDController(
            TiltConstants.P_GAIN,
            TiltConstants.I_GAIN,
            TiltConstants.D_GAIN,
            TiltConstants.CONSTRAINTS),
        ShooterConstants.INTAKE_ANGLE.getRadians());

    m_controller.setTolerance(TiltConstants.ANGLE_TOLERANCE.getRadians());

    left.restoreFactoryDefaults();
    right.restoreFactoryDefaults();

    left.setSmartCurrentLimit(TiltConstants.CURRENT_LIMIT);
    right.setSmartCurrentLimit(TiltConstants.CURRENT_LIMIT);
    left.setIdleMode(TiltConstants.IDLE_MODE);
    right.setIdleMode(TiltConstants.IDLE_MODE);

    right.follow(left);

    encoder.setPositionConversionFactor(TiltConstants.POSITION_CONVERSION_FACTOR);
    encoder.setVelocityConversionFactor(TiltConstants.VELOCITY_CONVERSION_FACTOR);

    encoder.setInverted(false);

    left.burnFlash();
    right.burnFlash();

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

  public Command setAngle(Rotation2d angle) {
    return Commands.runOnce(() -> this.setGoal(angle.getRadians()))
        .andThen(
            Commands.waitUntil(m_controller::atGoal).deadlineWith(Commands.run(() -> {}, this)));
  }

  @Override
  public void periodic() {
    super.periodic();

    SmartDashboard.putNumber("shooter/angle/current", encoder.getPosition());
    SmartDashboard.putNumber("shooter/angle/target", m_controller.getGoal().position);
  }
}
