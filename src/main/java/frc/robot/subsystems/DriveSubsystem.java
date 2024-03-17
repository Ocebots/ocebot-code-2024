// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.CANMappings;
import frc.constants.DriveConstants;
import frc.constants.VisionConstants;
import frc.robot.auto.Vision;
import frc.utils.SwerveUtils;
import org.photonvision.PhotonPoseEstimator;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft =
      new MAXSwerveModule(
          CANMappings.FRONT_LEFT_DRIVING,
          CANMappings.FRONT_LEFT_TURNING,
          DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule frontRight =
      new MAXSwerveModule(
          CANMappings.FRONT_RIGHT_DRIVING,
          CANMappings.FRONT_RIGHT_TURNING,
          DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule rearLeft =
      new MAXSwerveModule(
          CANMappings.REAR_LEFT_DRIVING,
          CANMappings.REAR_LEFT_TURNING,
          DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MAXSwerveModule rearRight =
      new MAXSwerveModule(
          CANMappings.REAR_RIGHT_DRIVING,
          CANMappings.REAR_RIGHT_TURNING,
          DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final AHRS gyro = new AHRS();

  private final Field2d field = new Field2d();

  private double currentRotation = 0.0;

  // Slew rate filter variables for controlling lateral acceleration
  private double currentTranslationDir = 0.0; // radian
  private double currentTranslationMag = 0.0; // meter per second
  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.MAX_ACCELERATION);

  private double prevTime = MathSharedStore.getTimestamp();

  private SlewRateLimiter rotLimiter =
      new SlewRateLimiter(DriveConstants.MAX_ROTATIONAL_ACCELERATION);

  private PhotonPoseEstimator vision =
      new PhotonPoseEstimator(
          VisionConstants.LAYOUT,
          VisionConstants.STRATEGY,
          Vision.getCamera(),
          VisionConstants.CAMERA_POSITION);

  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.DRIVE_KINEMATICS,
          getHeading(),
          new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.rearLeft.getPosition(),
            this.rearRight.getPosition()
          },
          new Pose2d(0, 0, getHeading()));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putData(field);
  }

  public void logData() {
    frontLeft.sendData("drive/frontLeft");
    frontRight.sendData("drive/frontRight");
    rearLeft.sendData("drive/rearLeft");
    rearRight.sendData("drive/rearRight");

    SmartDashboard.putNumber("drive/gyro", getHeading().getRadians());

    field.setRobotPose(getPose());

    // SmartDashboard.putNumber("drive/odometry/x",
    // odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("drive/odometry/y",
    // odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber(
    // "drive/odometry/rot", odometry.getPoseMeters().getRotation().getRadians());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    this.poseEstimator.update(
        getHeading(),
        new SwerveModulePosition[] {
          this.frontLeft.getPosition(),
          this.frontRight.getPosition(),
          this.rearLeft.getPosition(),
          this.rearRight.getPosition()
        });

    vision.setReferencePose(this.poseEstimator.getEstimatedPosition());

    vision
        .update()
        .ifPresent(
            (pose) ->
                this.poseEstimator.addVisionMeasurement(
                    pose.estimatedPose.toPose2d(), pose.timestampSeconds));
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.poseEstimator.getEstimatedPosition();
  }

  /**
   * Method to drive the robot using joystick info. This method may not work as intended if the
   * joystick does not move in a circle
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   * @param rateLimit Whether to enable rate limiting for smoother control.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar(theta and magnitude) for rate limiting
      double inputTranslationDir = SwerveUtils.wrapAngle(Math.atan2(ySpeed, xSpeed));
      double inputTranslationMag =
          Math.min(
              Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2))
                  * DriveConstants.MAX_SPEED_METERS_PER_SECOND,
              DriveConstants.MAX_SPEED_METERS_PER_SECOND);

      // If either translation magnitudes are low enough, it's direction is irrelevant
      if (currentTranslationMag < 0.1) {
        currentTranslationDir = inputTranslationDir;
      } else if (inputTranslationMag < 0.1) {
        inputTranslationDir = currentTranslationDir;
      }

      // If the new direction is over 90º from the current direction, slow the robot
      // to a near stop before continuing. The block above activates when the robot
      // has slowed down enough
      if (SwerveUtils.angleDifference(inputTranslationDir, currentTranslationDir) > Math.PI / 2.0) {
        inputTranslationDir = currentTranslationDir;
        inputTranslationMag *= -1;
      }

      // Decrease the requested speed based on how far the current translation
      // direction is from the requested one. A 90º angle means the requested
      // magnitude is 0, a 0º angle means the magnitude is unchanged.
      inputTranslationMag *=
          1
              - SwerveUtils.angleDifference(inputTranslationDir, currentTranslationDir)
                  / (Math.PI / 2);

      // Set the direction limit to a percent of the maxiumum based on the percent of
      // max speed we are currently going at. This is limited to prevent fully
      // disabling steering and leaves it at 10%.
      double dirLimit =
          Math.max(
              0,
              DriveConstants.MAX_DRIVE_ANGULAR_VELOCITY
                  * ((1 - currentTranslationMag / DriveConstants.MAX_SPEED_METERS_PER_SECOND) * 0.30
                      + 0.70));

      double currentTime = MathSharedStore.getTimestamp();

      this.currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      this.currentTranslationDir =
          SwerveUtils.stepTowardsCircular(
              currentTranslationDir, inputTranslationDir, dirLimit * (currentTime - prevTime));

      prevTime = currentTime;
      xSpeedCommanded = this.currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = this.currentTranslationMag * Math.sin(currentTranslationDir);
      this.currentRotation = rotLimiter.calculate(rot) * DriveConstants.MAX_ANGULAR_SPEED;
    } else {
      xSpeedCommanded = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
      ySpeedCommanded = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
      this.currentRotation = rot * DriveConstants.MAX_ANGULAR_SPEED;
    }

    var swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedCommanded,
                    ySpeedCommanded,
                    this.currentRotation,
                    getPose()
                        .getRotation()
                        .plus(
                            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                ? Rotation2d.fromDegrees(0)
                                : Rotation2d.fromDegrees(180)))
                : new ChassisSpeeds(xSpeedCommanded, ySpeedCommanded, this.currentRotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);

    this.frontLeft.setDesiredState(swerveModuleStates[0]);
    this.frontRight.setDesiredState(swerveModuleStates[1]);
    this.rearLeft.setDesiredState(swerveModuleStates[2]);
    this.rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    this.frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    this.frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    this.rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    this.frontLeft.setDesiredState(desiredStates[0]);
    this.frontRight.setDesiredState(desiredStates[1]);
    this.rearLeft.setDesiredState(desiredStates[2]);
    this.rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    this.frontLeft.resetEncoders();
    this.rearLeft.resetEncoders();
    this.frontRight.resetEncoders();
    this.rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    this.gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  private Rotation2d getHeading() {
    return Rotation2d.fromDegrees(
        this.gyro.getAngle() * (DriveConstants.GYRO_IS_REVERSED ? -1.0 : 1.0));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return this.gyro.getRate() * (DriveConstants.GYRO_IS_REVERSED ? -1.0 : 1.0);
  }

  public Command alignWithHeading(Rotation2d angle) {
    ProfiledPIDController controller =
        new ProfiledPIDController(
            DriveConstants.TURN_P_GAIN,
            DriveConstants.TURN_I_GAIN,
            DriveConstants.TURN_D_GAIN,
            DriveConstants.TURN_CONSTRAINTS);

    controller.setTolerance(Rotation2d.fromDegrees(2).getRadians());
    controller.enableContinuousInput(-Math.PI, Math.PI);

    return new ProfiledPIDCommand(
            controller,
            () -> MathUtil.angleModulus(getPose().getRotation().getRadians()),
            () -> new TrapezoidProfile.State(MathUtil.angleModulus(angle.getRadians()), 0),
            (value, _targetState) -> this.drive(0, 0, value, false, false),
            this)
        .until(() -> controller.atGoal());
  }
}
