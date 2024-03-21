package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.constants.CANMappings;
import frc.constants.ShooterConstants;
import frc.constants.ShooterConstants.FlywheelConstants;
import frc.constants.ShooterConstants.IntermediateConstants;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HeightSubsystem;
import frc.robot.subsystems.shooter.TiltSubsystem;
import java.util.HashSet;

public class ShootSubsystem extends SubsystemBase {
  private final CANSparkMax intermediate =
      new CANSparkMax(CANMappings.INTERMEDIATE, MotorType.kBrushless);

  private final RelativeEncoder intermediateEncoder = intermediate.getEncoder();

  private final DigitalInput limitSwitch = new DigitalInput(0);

  private final TiltSubsystem tiltSubsystem = new TiltSubsystem();
  private final HeightSubsystem heightSubsystem = new HeightSubsystem();

  private FlywheelSubsystem leftShooter =
      new FlywheelSubsystem(
          CANMappings.SHOOTER_LEFT,
          false,
          FlywheelConstants.LEFT_P_GAIN,
          FlywheelConstants.LEFT_STATIC_GAIN,
          FlywheelConstants.LEFT_VELOCITY_GAIN,
          FlywheelConstants.LEFT_ACCELERATION_GAIN);
  private FlywheelSubsystem rightShooter =
      new FlywheelSubsystem(
          CANMappings.SHOOTER_RIGHT,
          true,
          FlywheelConstants.RIGHT_P_GAIN,
          FlywheelConstants.RIGHT_STATIC_GAIN,
          FlywheelConstants.RIGHT_VELOCITY_GAIN,
          FlywheelConstants.RIGHT_ACCELERATION_GAIN);

  public ShootSubsystem() {
    super();

    leftShooter.setName("left");
    rightShooter.setName("right");

    intermediate.restoreFactoryDefaults();

    intermediate.setIdleMode(IntermediateConstants.IDLE_MODE);
    intermediate.setSmartCurrentLimit(IntermediateConstants.CURRENT_LIMIT);

    intermediate.setInverted(true);

    intermediateEncoder.setPositionConversionFactor(
        IntermediateConstants.POSITION_CONVERSION_FACTOR);
    intermediateEncoder.setVelocityConversionFactor(
        IntermediateConstants.VELOCITY_CONVERSION_FACTOR);

    intermediateEncoder.setPosition(0.0);

    intermediate.burnFlash();

    this.setDefaultCommand(
        setHeightAndTilt(ShooterConstants.INTAKE_HEIGHT, ShooterConstants.INTAKE_ANGLE));
  }

  public Command adjust(boolean isForward) {
    return Commands.runEnd(
        () -> intermediate.set(0.3 * (isForward ? 1 : -1)), () -> intermediate.set(0.0), this);
  }

  @Override
  public void periodic() {
    logInfo();
  }

  public void logInfo() {
    SmartDashboard.putNumber("shooter/inter/current", intermediateEncoder.getPosition());
    SmartDashboard.putBoolean("shooter/inter/detected", limitSwitch.get());
  }

  /**
   * Sets the velocity of each wheel and waits for it to reach it. Then run the shoot speed for a
   * set amount of time. This does not rely on this subsystem because it is internal.
   *
   * @param velocity The desired velocity of the note as it exits the schooter
   */
  private Command shoot(double velocity) {
    SmartDashboard.putNumber("shooter/velocity/desired", velocity);
    return Commands.race(
        leftShooter.setVelocity(velocity),
        rightShooter.setVelocity(velocity),
        Commands.parallel(
                leftShooter.waitForVelocity(velocity), rightShooter.waitForVelocity(velocity))
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

  /** Move the intake into the correct position and configure the intermediate motor for intaking */
  public Command intakeMode() {
    return setHeightAndTilt(ShooterConstants.INTAKE_HEIGHT, ShooterConstants.INTAKE_ANGLE);
  }

  /** Wait until a note has been detected by the intermediate motor */
  public Command waitForIntake() {
    return Commands.runEnd(() -> intermediate.set(1), () -> intermediate.set(0.0))
        .raceWith(Commands.waitUntil(() -> limitSwitch.get()));
  }

  /** Move the note into the correct position within the robot */
  public Command completeIntake() {
    return Commands.runEnd(() -> intermediate.set(-0.2), () -> intermediate.set(0))
        .withTimeout(0.2);
  }

  /** Set the height and tilt of the shooter. This command does require the current subsystem */
  private Command setHeightAndTilt(double height, Rotation2d angle) {
    return Commands.parallel(heightSubsystem.setHeight(height), tiltSubsystem.setAngle(angle))
        .raceWith(Commands.run(() -> {}, this));
  }

  /** Move the shooter and shoot into the amp */
  public Command scoreAmp() {
    return setHeightAndTilt(ShooterConstants.AMP_HEIGHT, ShooterConstants.AMP_ANGLE)
        .andThen(shoot(ShooterConstants.AMP_SPEED));
  }

  public Command scoreProtectedInner() {
    return setHeightAndTilt(0.254, Rotation2d.fromDegrees(203)).andThen(shoot(24));
  }

  /**
   * Using the current position of the robot, score a note into the speaker. If that is not
   * possible, do nothing
   */
  private Command scoreSpeakerInner(DriveSubsystem drive) {
    Pose2d speakerPose =
        DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? new Pose2d(0.229997, 5.54787, Rotation2d.fromDegrees(0))
            : new Pose2d(16.5412 - 0.229997, 5.54787, Rotation2d.fromDegrees(0));

    HashSet<Subsystem> subsystems = new HashSet<>();
    subsystems.add(this);

    return drive
        .alignWithHeading(
            () -> speakerPose.getTranslation().minus(drive.getPose().getTranslation()).getAngle())
        .alongWith(
            Commands.defer(
                () -> {
                  double distance =
                      speakerPose.getTranslation().getDistance(drive.getPose().getTranslation());

                  Rotation2d angle =
                      Rotation2d.fromDegrees(
                          ShooterConstants.AUTOAIM_GAIN
                              * Math.pow(distance, ShooterConstants.AUTOAIM_EXPONENT));

                  return setHeightAndTilt(ShooterConstants.SPEAKER_SCORE_HEIGHT, angle);
                },
                subsystems))
        .andThen(shoot(24));
  }

  public Command climb() {
    return null; // TODO: Climb onto the chain
  }

  public Command sysId() {
    SysIdRoutine routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  leftShooter.motor.setVoltage(voltage.in(Volts));
                  rightShooter.motor.setVoltage(voltage.in(Volts));
                },
                (logger) -> {
                  logger
                      .motor("left")
                      .voltage(
                          Volts.of(
                              leftShooter.motor.getBusVoltage()
                                  * leftShooter.motor.getAppliedOutput()))
                      .linearPosition(Meters.of(leftShooter.encoder.getPosition()))
                      .linearVelocity(MetersPerSecond.of(leftShooter.encoder.getVelocity()));

                  logger
                      .motor("right")
                      .voltage(
                          Volts.of(
                              rightShooter.motor.getBusVoltage()
                                  * rightShooter.motor.getAppliedOutput()))
                      .linearPosition(Meters.of(rightShooter.encoder.getPosition()))
                      .linearVelocity(MetersPerSecond.of(rightShooter.encoder.getVelocity()));
                },
                this));

    return routine
        .dynamic(Direction.kForward)
        .andThen(
            routine.dynamic(Direction.kReverse),
            routine.quasistatic(Direction.kForward),
            routine.quasistatic(Direction.kReverse));
  }

  public Command scoreSpeaker(DriveSubsystem driveSubsystem) {
    HashSet<Subsystem> subsystems = new HashSet<>();

    subsystems.add(this);
    subsystems.add(driveSubsystem);

    return Commands.defer(() -> scoreSpeakerInner(driveSubsystem), subsystems);
  }

  public Command scoreProtected() {
    HashSet<Subsystem> subsystems = new HashSet<>();

    subsystems.add(this);

    return Commands.defer(() -> scoreProtectedInner(), subsystems);
  }
}
