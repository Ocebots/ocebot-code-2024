package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.CANMappings;
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

  private Rotation2d currentTargetAngle = Rotation2d.fromDegrees(0);
  private double angleStartTime = 0;

  public ShootSubsystem() {
    super();
    leftTilt.follow(rightTilt);

    // rotations to radians
    tiltEncoder.setPositionConversionFactor(Math.PI * 2);
    tiltEncoder.setVelocityConversionFactor(Math.PI * 2);

    ArmFeedforward armFeedforward =
        new ArmFeedforward(
            TiltConstants.STATIC_GAIN, TiltConstants.GRAVITY_GAIN, TiltConstants.VELOCITY_GAIN);

    PIDController tiltController =
        new PIDController(TiltConstants.P_GAIN, TiltConstants.I_GAIN, TiltConstants.D_GAIN);

    TrapezoidProfile tiltProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                TiltConstants.MAX_ANGULAR_VELOCITY, TiltConstants.MAX_ANGULAR_ACCELERATION));

    this.setDefaultCommand(
        Commands.run(
            () -> {
              TrapezoidProfile.State desiredState =
                  tiltProfile.calculate(
                      Timer.getFPGATimestamp() - angleStartTime,
                      new TrapezoidProfile.State(
                          tiltEncoder.getPosition(), tiltEncoder.getVelocity()),
                      new TrapezoidProfile.State(currentTargetAngle.getRadians(), 0));

              rightTilt.setVoltage(
                  tiltController.calculate(tiltEncoder.getPosition(), desiredState.position)
                      + armFeedforward.calculate(desiredState.position, desiredState.velocity));
            }));
  }

  /*
   * @param velocity meters per second of the ring
   */
  private Command shoot(double velocity) {
    return null; // TODO: Make it shoot at the current angle
  }

  private Command setAngle(Rotation2d newAngle) {
    return Commands.runOnce(() -> this.currentTargetAngle = newAngle)
        .andThen(
            Commands.waitUntil(
                () ->
                    tiltEncoder.getPosition()
                            >= currentTargetAngle.getRadians()
                                - TiltConstants.ANGLE_TOLERANCE.getRadians()
                        && tiltEncoder.getPosition()
                            <= currentTargetAngle.getRadians()
                                + TiltConstants.ANGLE_TOLERANCE.getRadians()));
  }

  private Command setHeight(double meters) {
    return null; // TODO: Set the height of the arm to a certain amount
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
