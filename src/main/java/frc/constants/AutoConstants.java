package frc.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
  public static final double MAX_SPEED_METERS_PER_SECOND = 0.5;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

  public static final double PX_CONTROLLER = 1;
  public static final double PY_CONTROLLER = 1;
  public static final double P_THETA_CONTROLLER = 1;

  // Constraint for the motion profiled robot angle controller
  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
}
