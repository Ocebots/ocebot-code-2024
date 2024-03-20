package frc.constants;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
  public static final double MAX_SPEED_METERS_PER_SECOND = 1.5;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 10;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
  public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

  public static final double PX_CONTROLLER = 1;
  public static final double PY_CONTROLLER = 1;
  public static final double P_THETA_CONTROLLER = 1;

  // Constraint for the motion profiled robot angle controller
  public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
      new TrapezoidProfile.Constraints(
          MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

  public static final PathConstraints PATH_CONSTRAINTS =
      new PathConstraints(
          MAX_SPEED_METERS_PER_SECOND,
          MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
          MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
          MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

  public static final HolonomicPathFollowerConfig PATH_CONFIG =
      new HolonomicPathFollowerConfig(
          new PIDConstants(PX_CONTROLLER),
          new PIDConstants(P_THETA_CONTROLLER),
          DriveConstants.MAX_SPEED_METERS_PER_SECOND,
          0.0, // TODO: Find proper radius
          new ReplanningConfig());
}
