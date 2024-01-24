package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.CANMappings;
import frc.constants.ShooterConstants.HeightConstants;
import frc.constants.ShooterConstants.TiltConstants;

public class ShootSubsystem extends SubsystemBase {
  private CANSparkMax leftElevator =
      new CANSparkMax(CANMappings.ELEVATOR_LEFT, MotorType.kBrushless);
  private CANSparkMax rightElevator =
      new CANSparkMax(CANMappings.ELEVATOR_RIGHT, MotorType.kBrushless);

  private CANSparkMax leftShooter = new CANSparkMax(CANMappings.SHOOTER_LEFT, MotorType.kBrushless);
  private CANSparkMax rightShooter =
      new CANSparkMax(CANMappings.SHOOTER_RIGHT, MotorType.kBrushless);

  private CANSparkMax leftTilt = new CANSparkMax(CANMappings.TILT_LEFT, MotorType.kBrushless);
  private CANSparkMax rightTilt = new CANSparkMax(CANMappings.TILT_RIGHT, MotorType.kBrushless);

  private CANSparkMax intermediate =
      new CANSparkMax(CANMappings.INTERMEDIATE, MotorType.kBrushless);

  private AbsoluteEncoder tiltEncoder =
      rightTilt.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  private RelativeEncoder heightEncoder = rightElevator.getEncoder();

  private Rotation2d targetAngle = Rotation2d.fromDegrees(0);
  private double angleStartTime = 0;

  private double targetHeight = 0;
  private double heightStartTime = 0;

  public ShootSubsystem() {
    super();

    leftTilt.setSmartCurrentLimit(TiltConstants.CURRENT_LIMIT);
    rightTilt.setSmartCurrentLimit(TiltConstants.CURRENT_LIMIT);
    leftTilt.setIdleMode(TiltConstants.IDLE_MODE);
    rightTilt.setIdleMode(TiltConstants.IDLE_MODE);

    leftElevator.setSmartCurrentLimit(HeightConstants.CURRENT_LIMIT);
    rightElevator.setSmartCurrentLimit(HeightConstants.CURRENT_LIMIT);
    leftElevator.setIdleMode(HeightConstants.IDLE_MODE);
    rightElevator.setIdleMode(HeightConstants.IDLE_MODE);

    leftTilt.follow(rightTilt);
    leftElevator.follow(rightElevator);

    // rotations to radians
    tiltEncoder.setPositionConversionFactor(TiltConstants.POSITION_CONVERSION_FACTOR);
    tiltEncoder.setVelocityConversionFactor(TiltConstants.VELOCITY_CONVERSION_FACTOR);

    heightEncoder.setPositionConversionFactor(HeightConstants.POSITION_CONVERSION_FACTOR);
    heightEncoder.setVelocityConversionFactor(HeightConstants.VELOCITY_CONVERSION_FACTOR);

    targetAngle = Rotation2d.fromRadians(tiltEncoder.getPosition());

    ArmFeedforward armFeedforward =
        new ArmFeedforward(
            TiltConstants.STATIC_GAIN, TiltConstants.GRAVITY_GAIN, TiltConstants.VELOCITY_GAIN);

    PIDController tiltController =
        new PIDController(TiltConstants.P_GAIN, TiltConstants.I_GAIN, TiltConstants.D_GAIN);

    TrapezoidProfile tiltProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                TiltConstants.MAX_ANGULAR_VELOCITY, TiltConstants.MAX_ANGULAR_ACCELERATION));

    Command tiltCommand =
        Commands.run(
            () -> {
              double currentAngle = tiltEncoder.getPosition();

              TrapezoidProfile.State desiredState =
                  tiltProfile.calculate(
                      Timer.getFPGATimestamp() - angleStartTime,
                      new TrapezoidProfile.State(currentAngle, tiltEncoder.getVelocity()),
                      new TrapezoidProfile.State(targetAngle.getRadians(), 0));

              rightTilt.setVoltage(
                  tiltController.calculate(currentAngle, desiredState.position)
                      + armFeedforward.calculate(desiredState.position, desiredState.velocity));
            });

    ElevatorFeedforward elevatorFeedforward =
        new ElevatorFeedforward(
            HeightConstants.STATIC_GAIN,
            HeightConstants.GRAVITY_GAIN,
            HeightConstants.VELOCITY_GAIN);

    PIDController heightController =
        new PIDController(HeightConstants.P_GAIN, HeightConstants.I_GAIN, HeightConstants.D_GAIN);

    TrapezoidProfile heightProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                HeightConstants.MAX_VELOCITY, HeightConstants.MAX_ACCELERATION));

    Command heightCommand =
        Commands.run(
            () -> {
              double currentPosition = heightEncoder.getPosition();

              TrapezoidProfile.State desiredState =
                  heightProfile.calculate(
                      Timer.getFPGATimestamp() - heightStartTime,
                      new TrapezoidProfile.State(currentPosition, heightEncoder.getVelocity()),
                      new TrapezoidProfile.State(targetHeight, 0));

              rightElevator.setVoltage(
                  heightController.calculate(currentPosition, desiredState.position)
                      + elevatorFeedforward.calculate(desiredState.velocity));
            });

    this.setDefaultCommand(tiltCommand.alongWith(heightCommand));
  }

  /*
   * @param velocity meters per second of the ring
   */
  private Command shoot(double velocity) {
    return null; // TODO: Make it shoot at the current angle
  }

  private Command setAngle(Rotation2d newAngle) {
    return Commands.runOnce(
            () -> {
              this.targetAngle = newAngle;
              this.angleStartTime = Timer.getFPGATimestamp();
            })
        .andThen(
            Commands.waitUntil(
                () -> {
                  double currentAngle = tiltEncoder.getPosition();

                  return currentAngle
                          >= targetAngle.getRadians() - TiltConstants.ANGLE_TOLERANCE.getRadians()
                      && currentAngle
                          <= targetAngle.getRadians() + TiltConstants.ANGLE_TOLERANCE.getRadians();
                }));
  }

  private Command setHeight(double meters) {
    return Commands.runOnce(
            () -> {
              this.targetHeight = meters;
              this.heightStartTime = Timer.getFPGATimestamp();
            })
        .andThen(
            Commands.waitUntil(
                () -> {
                  double currentPosition = heightEncoder.getPosition();

                  return currentPosition >= targetHeight - HeightConstants.TOLERANCE
                      && currentPosition <= targetHeight + HeightConstants.TOLERANCE;
                }));
  }

  public Command underStageMode() {
    return null; // TODO: Set the angle and height to be able to go under the stage
  }

  public Command scoreAmp() {
    return null; // TODO: Set the angle and height to go under the stage
  }

  public Command scoreSpeaker(Pose2d currentPos) {
    return null; // TODO: Using the current posistion of the robot, score in speaker. If unable,
    // do nothing
  }

  public Command climb() {
    return null; // TODO: Climb onto the chain
  }
}
