// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.constants.ModuleConstants;
import frc.constants.PIDConstants;

public class MAXSwerveModule {
  private final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;

  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    this.drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    this.turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    this.drivingSparkMax.restoreFactoryDefaults();
    this.turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    this.drivingEncoder = drivingSparkMax.getEncoder();
    this.turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    this.drivingPIDController = drivingSparkMax.getPIDController();
    this.turningPIDController = turningSparkMax.getPIDController();
    this.drivingPIDController.setFeedbackDevice(drivingEncoder);
    this.turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    this.drivingEncoder.setPositionConversionFactor(
        ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR);
    this.drivingEncoder.setVelocityConversionFactor(
        ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    this.turningEncoder.setPositionConversionFactor(
        ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
    this.turningEncoder.setVelocityConversionFactor(
        ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    this.turningEncoder.setInverted(ModuleConstants.IS_TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    this.turningPIDController.setPositionPIDWrappingEnabled(true);
    this.turningPIDController.setPositionPIDWrappingMinInput(
        PIDConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
    this.turningPIDController.setPositionPIDWrappingMaxInput(
        PIDConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    this.drivingPIDController.setP(PIDConstants.DRIVING_P);
    this.drivingPIDController.setI(PIDConstants.DRIVING_I);
    this.drivingPIDController.setD(PIDConstants.DRIVING_D);
    this.drivingPIDController.setFF(PIDConstants.DRIVING_FF);
    this.drivingPIDController.setOutputRange(
        PIDConstants.DRIVING_MIN_OUTPUT, PIDConstants.DRIVING_MAX_OUTPUT);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    this.turningPIDController.setP(PIDConstants.TURNING_P);
    this.turningPIDController.setI(PIDConstants.TURNING_I);
    this.turningPIDController.setD(PIDConstants.TURNING_D);
    this.turningPIDController.setFF(PIDConstants.TURNING_FF);
    this.turningPIDController.setOutputRange(
        PIDConstants.TURNING_MIN_OUTPUT, PIDConstants.TURNING_MAX_OUTPUT);

    this.drivingSparkMax.setIdleMode(ModuleConstants.DRIVING_MOTOR_IDLE_MODE);
    this.turningSparkMax.setIdleMode(ModuleConstants.TURNING_MOTOR_IDLE_MODE);
    this.drivingSparkMax.setSmartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT);
    this.turningSparkMax.setSmartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    this.drivingSparkMax.burnFlash();
    this.turningSparkMax.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    this.desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    this.drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        this.drivingEncoder.getVelocity(),
        new Rotation2d(this.turningEncoder.getPosition() - this.chassisAngularOffset));
  }

  public SwerveModuleState getDesiredState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        this.desiredState.speedMetersPerSecond,
        new Rotation2d(this.desiredState.angle.getRadians()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        this.drivingEncoder.getPosition(),
        new Rotation2d(this.turningEncoder.getPosition() - this.chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle =
        desiredState.angle.plus(Rotation2d.fromRadians(this.chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(
            correctedDesiredState, new Rotation2d(this.turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    this.drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    this.turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    this.drivingEncoder.setPosition(0);
  }

  public void sendData(
      String
          module_path) { // TODO: Make the error output easier to understand, actually, probably not
    SmartDashboard.putNumber(
        module_path + "/turn/error",
        this.getDesiredState().angle.getDegrees() - this.getState().angle.getDegrees());
    SmartDashboard.putNumber(
        module_path + "/drive/error",
        this.getDesiredState().speedMetersPerSecond - this.getState().speedMetersPerSecond);

    SmartDashboard.putNumber(module_path + "/turn/measured", this.getState().angle.getDegrees());
    SmartDashboard.putNumber(module_path + "/drive/measured", this.getState().speedMetersPerSecond);

    SmartDashboard.putNumber(
        module_path + "/turn/commanded", this.getDesiredState().angle.getDegrees());
    SmartDashboard.putNumber(
        module_path + "/drive/commanded", this.getDesiredState().speedMetersPerSecond);
  }
}
