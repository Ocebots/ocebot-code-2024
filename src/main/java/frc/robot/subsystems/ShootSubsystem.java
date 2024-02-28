package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.CANMappings;
import frc.constants.ShooterConstants;
import frc.constants.ShooterConstants.HeightConstants;
import frc.constants.ShooterConstants.IntermediateConstants;
import frc.constants.ShooterConstants.ShooterMotorConstants;
import frc.constants.ShooterConstants.TiltConstants;

public class ShootSubsystem extends SubsystemBase {
  private CANSparkMax leftElevator =
      new CANSparkMax(CANMappings.ELEVATOR_LEFT, MotorType.kBrushless);
  private CANSparkMax rightElevator =
      new CANSparkMax(CANMappings.ELEVATOR_RIGHT, MotorType.kBrushless);

  private CANSparkFlex leftShooter =
      new CANSparkFlex(CANMappings.SHOOTER_LEFT, MotorType.kBrushless);
  private CANSparkFlex rightShooter =
      new CANSparkFlex(CANMappings.SHOOTER_RIGHT, MotorType.kBrushless);

  private CANSparkMax leftTilt = new CANSparkMax(CANMappings.TILT_LEFT, MotorType.kBrushless);
  private CANSparkMax rightTilt = new CANSparkMax(CANMappings.TILT_RIGHT, MotorType.kBrushless);

  private CANSparkMax intermediate =
      new CANSparkMax(CANMappings.INTERMEDIATE, MotorType.kBrushless);

  private AbsoluteEncoder tiltEncoder =
      leftTilt.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

  private RelativeEncoder heightEncoder = leftElevator.getEncoder();

  private RelativeEncoder leftShooterEncoder = leftShooter.getEncoder();
  private RelativeEncoder rightShooterEncoder = rightShooter.getEncoder();

  private RelativeEncoder intermediateEncoder = intermediate.getEncoder();

  private DigitalInput limitSwitch = new DigitalInput(0);

  private Rotation2d targetAngle = Rotation2d.fromDegrees(0);
  private double angleStartTime = 0;

  private double targetHeight = 0;
  private double heightStartTime = 0;

  TrapezoidProfile heightProfile;
  ElevatorFeedforward elevatorFeedforward;
  PIDController heightController;

  ArmFeedforward armFeedforward;
  PIDController tiltController;
  TrapezoidProfile tiltProfile;

  public ShootSubsystem() {
    super();

    leftTilt.restoreFactoryDefaults();
    rightTilt.restoreFactoryDefaults();
    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    leftElevator.restoreFactoryDefaults();
    rightElevator.restoreFactoryDefaults();
    intermediate.restoreFactoryDefaults();

    leftTilt.setSmartCurrentLimit(TiltConstants.CURRENT_LIMIT);
    rightTilt.setSmartCurrentLimit(TiltConstants.CURRENT_LIMIT);
    leftTilt.setIdleMode(TiltConstants.IDLE_MODE);
    rightTilt.setIdleMode(TiltConstants.IDLE_MODE);

    leftElevator.setSmartCurrentLimit(HeightConstants.CURRENT_LIMIT);
    rightElevator.setSmartCurrentLimit(HeightConstants.CURRENT_LIMIT);
    leftElevator.setIdleMode(HeightConstants.IDLE_MODE);
    rightElevator.setIdleMode(HeightConstants.IDLE_MODE);

    leftShooter.setSmartCurrentLimit(ShooterMotorConstants.CURRENT_LIMIT);
    rightShooter.setSmartCurrentLimit(ShooterMotorConstants.CURRENT_LIMIT);
    leftShooter.setIdleMode(ShooterMotorConstants.IDLE_MODE);
    rightShooter.setIdleMode(ShooterMotorConstants.IDLE_MODE);

    intermediate.setIdleMode(IntermediateConstants.IDLE_MODE);
    intermediate.setSmartCurrentLimit(IntermediateConstants.CURRENT_LIMIT);

    intermediate.setInverted(true);

    intermediateEncoder.setPositionConversionFactor(
        IntermediateConstants.POSITION_CONVERSION_FACTOR);
    intermediateEncoder.setVelocityConversionFactor(
        IntermediateConstants.VELOCITY_CONVERSION_FACTOR);

    intermediateEncoder.setPosition(0.0);

    leftShooter.setInverted(false);
    rightShooter.setInverted(true);

    leftShooterEncoder.setPositionConversionFactor(
        ShooterMotorConstants.POSITION_CONVERSION_FACTOR);
    leftShooterEncoder.setVelocityConversionFactor(
        ShooterMotorConstants.VELOCITY_CONVERSION_FACTOR);
    rightShooterEncoder.setPositionConversionFactor(
        ShooterMotorConstants.POSITION_CONVERSION_FACTOR);
    rightShooterEncoder.setVelocityConversionFactor(
        ShooterMotorConstants.VELOCITY_CONVERSION_FACTOR);

    rightTilt.follow(leftTilt);
    rightElevator.follow(leftElevator);

    tiltEncoder.setPositionConversionFactor(TiltConstants.POSITION_CONVERSION_FACTOR);
    tiltEncoder.setVelocityConversionFactor(TiltConstants.VELOCITY_CONVERSION_FACTOR);

    tiltEncoder.setInverted(false);

    heightEncoder.setPositionConversionFactor(HeightConstants.POSITION_CONVERSION_FACTOR);
    heightEncoder.setVelocityConversionFactor(HeightConstants.VELOCITY_CONVERSION_FACTOR);

    targetAngle = Rotation2d.fromRadians(tiltEncoder.getPosition());
    targetHeight = heightEncoder.getPosition();

    leftTilt.burnFlash();
    rightTilt.burnFlash();
    leftShooter.burnFlash();
    rightShooter.burnFlash();
    leftElevator.burnFlash();
    rightElevator.burnFlash();
    intermediate.burnFlash();

    armFeedforward =
        new ArmFeedforward(
            TiltConstants.STATIC_GAIN, TiltConstants.GRAVITY_GAIN, TiltConstants.VELOCITY_GAIN);

    tiltController =
        new PIDController(TiltConstants.P_GAIN, TiltConstants.I_GAIN, TiltConstants.D_GAIN);

    tiltProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                TiltConstants.MAX_ANGULAR_VELOCITY, TiltConstants.MAX_ANGULAR_ACCELERATION));

    elevatorFeedforward =
        new ElevatorFeedforward(
            HeightConstants.STATIC_GAIN,
            HeightConstants.GRAVITY_GAIN,
            HeightConstants.VELOCITY_GAIN);

    heightController =
        new PIDController(HeightConstants.P_GAIN, HeightConstants.I_GAIN, HeightConstants.D_GAIN);

    heightProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                HeightConstants.MAX_VELOCITY, HeightConstants.MAX_ACCELERATION));
  }

  public double getAngleRads() {
    return this.tiltEncoder.getPosition();
  }

  public double getHeight() {
    return this.heightEncoder.getPosition();
  }

  @Override
  public void periodic() {
    double currentPosition = heightEncoder.getPosition();

    TrapezoidProfile.State heightDesiredState =
        heightProfile.calculate(
            Timer.getFPGATimestamp() - heightStartTime,
            new TrapezoidProfile.State(currentPosition, heightEncoder.getVelocity()),
            new TrapezoidProfile.State(targetHeight, 0));

    leftElevator.setVoltage(
        heightController.calculate(currentPosition, heightDesiredState.position)
            + elevatorFeedforward.calculate(heightDesiredState.velocity));

    double currentAngle = tiltEncoder.getPosition();

    TrapezoidProfile.State tiltDesiredState =
        tiltProfile.calculate(
            Timer.getFPGATimestamp() - angleStartTime,
            new TrapezoidProfile.State(currentAngle, tiltEncoder.getVelocity()),
            new TrapezoidProfile.State(targetAngle.getRadians(), 0));

    leftTilt.setVoltage(
        tiltController.calculate(currentAngle, tiltDesiredState.position)
            + armFeedforward.calculate(tiltDesiredState.position, tiltDesiredState.velocity));

    logInfo();
  }

  public void logInfo() {
    SmartDashboard.putNumber("shooter/velocity/left", leftShooterEncoder.getVelocity());
    SmartDashboard.putNumber("shooter/velocity/right", rightShooterEncoder.getVelocity());

    SmartDashboard.putNumber("shooter/angle/current", tiltEncoder.getPosition());
    SmartDashboard.putNumber("shooter/angle/target", targetAngle.getRadians());
    SmartDashboard.putNumber("shooter/height/current", heightEncoder.getPosition());
    SmartDashboard.putNumber("shooter/height/target", targetHeight);

    SmartDashboard.putNumber("shooter/inter/current", intermediateEncoder.getPosition());
    SmartDashboard.putBoolean("shooter/inter/detected", limitSwitch.get());
  }

  public void setHeightRaw(double height) {
    this.targetHeight = height;
    this.heightStartTime = Timer.getFPGATimestamp();
  }

  public void setAngleRaw(Rotation2d angle) {
    this.targetAngle = angle;
    this.angleStartTime = Timer.getFPGATimestamp();
  }

  /**
   * Control the velocity of one motor
   *
   * @param velocity The desired velocity in m/s
   * @param motor The motor to control
   * @param encoder The encoder to use to measure velocity
   */
  private Command setVelocityOneSide(double velocity, CANSparkFlex motor, RelativeEncoder encoder) {
    PIDController pidController =
        new PIDController(
            ShooterMotorConstants.P_GAIN,
            ShooterMotorConstants.I_GAIN,
            ShooterMotorConstants.D_GAIN);

    pidController.setSetpoint(velocity);

    SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(
            ShooterMotorConstants.STATIC_GAIN, ShooterMotorConstants.VELOCITY_GAIN);

    return Commands.runEnd(
        () -> {
          motor.setVoltage(
              pidController.calculate(encoder.getVelocity()) + feedforward.calculate(velocity));
        },
        () -> {
          motor.set(0);
          pidController.close();
        });
  }

  /** Wait until a motor hits */
  private Command waitForVelocityOneSide(double velocity, RelativeEncoder encoder) {
    return Commands.waitUntil(
        () -> {
          double currentVelocity = encoder.getVelocity();
          return currentVelocity > velocity;
        });
  }

  /**
   * Sets the velocity of each wheel and waits for it to reach it. Then run the shoot speed for a
   * set amount of time. This does not rely on this subsystem because it is internal.
   *
   * @param velocity The desired velocity of the note as it exits the schooter
   */
  public Command shoot(double velocity) {
    SmartDashboard.putNumber("shooter/velocity/desired", velocity);
    return Commands.race(
        setVelocityOneSide(velocity, leftShooter, leftShooterEncoder),
        setVelocityOneSide(velocity, rightShooter, rightShooterEncoder),
        Commands.parallel(
                waitForVelocityOneSide(velocity, leftShooterEncoder),
                waitForVelocityOneSide(velocity, rightShooterEncoder))
            .andThen(
                Commands.runEnd(
                        () -> intermediate.set(IntermediateConstants.SHOOT_SPEED),
                        () -> {
                          intermediate.set(0);
                          SmartDashboard.putBoolean("shooter/hasNote", false);
                        })
                    .withTimeout(IntermediateConstants.SHOOT_TIME)),
        Commands.run(() -> {}, this));
  }

  /**
   * Sets the angle of the arm and waits until it is set. If the angle is set by another command
   * before it is finished, the command will complete when the new target is reached. This command
   * does not rely on this subsystem because it is internal. Zero is horizontal forward and 90
   * degrees is up.
   *
   * @param newAngle The abosolute angle of the arm
   */
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

  /**
   * Sets the height of the arm and waits until it is set. If the height is set by another command
   * before it is finished, the command will complete when the new target is reached. This command
   * does not rely on this subsystem because it is internal
   *
   * @param meters The abosolute height in meters of the arm
   */
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

  /** Move the intake into the correct position and configure the intermediate motor for intaking */
  public Command intakeMode() {
    return Commands.runOnce(
        () -> {
          intermediateEncoder.setPosition(0);
          intermediate.setIdleMode(IdleMode.kCoast);
        },
        this);
  }

  /** Wait until a note has been detected by the intermediate motor */
  public Command waitForIntake() {
    return Commands.runEnd(() -> intermediate.set(1), () -> intermediate.set(0.0))
        .raceWith(Commands.waitUntil(() -> limitSwitch.get()));
  }

  /** Move the note into the correct positioon within the robot */
  public Command completeIntake() {
    PIDController controller =
        new PIDController(
            IntermediateConstants.P_GAIN,
            IntermediateConstants.I_GAIN,
            IntermediateConstants.D_GAIN);
    SimpleMotorFeedforward feedforward =
        new SimpleMotorFeedforward(
            IntermediateConstants.STATIC_GAIN, IntermediateConstants.VELOCITY_GAIN);
    TrapezoidProfile profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                IntermediateConstants.MAX_VELOCITY, IntermediateConstants.MAX_ACCELERATION));

    Timer timer = new Timer();
    timer.start();

    return Commands.runOnce(() -> intermediateEncoder.setPosition(0.0))
        .andThen(
            Commands.runEnd(
                    () -> {
                      double currentPosititon = intermediateEncoder.getPosition();
                      TrapezoidProfile.State desiredState =
                          profile.calculate(
                              timer.get(),
                              new TrapezoidProfile.State(
                                  currentPosititon, intermediateEncoder.getVelocity()),
                              new TrapezoidProfile.State(IntermediateConstants.FINAL_OFFSET, 0));

                      intermediate.setVoltage(
                          controller.calculate(currentPosititon, desiredState.position)
                              + feedforward.calculate(desiredState.velocity));
                    },
                    () -> {
                      intermediate.set(0);
                      controller.close();
                      SmartDashboard.putBoolean("shooter/hasNote", true);
                    },
                    this)
                .until(
                    () -> {
                      double currentPosition = intermediateEncoder.getPosition();

                      return currentPosition >= IntermediateConstants.FINAL_OFFSET;
                    }));
  }

  /** Set the height and tilt of the shooter. This command does require the current subsystem */
  private Command setHeightAndTilt(double height, Rotation2d angle) {
    return Commands.parallel(setHeight(height), setAngle(angle))
        .raceWith(Commands.run(() -> {}, this));
  }

  /** Move the shooter and shoot into the amp */
  public Command scoreAmp() {
    return setHeightAndTilt(ShooterConstants.AMP_HEIGHT, ShooterConstants.AMP_ANGLE)
        .andThen(shoot(ShooterConstants.AMP_SPEED));
  }

  /**
   * Using the current position of the robot, score a note into the speaker. If that is not
   * possible, do nothing
   */
  public Command scoreSpeaker(Pose2d currentPos) {
    return null; // TODO: Using the current posistion of the robot, score in speaker. If unable,
    // do nothing
  }

  public Command climb() {
    return null; // TODO: Climb onto the chain
  }
}
