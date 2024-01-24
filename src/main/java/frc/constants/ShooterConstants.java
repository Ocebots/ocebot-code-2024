package frc.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {
  public static class TiltConstants {
    public static double GRAVITY_GAIN = 0;
    public static double STATIC_GAIN = 0;
    public static double VELOCITY_GAIN = 0;

    public static double P_GAIN = 0;
    public static double I_GAIN = 0;
    public static double D_GAIN = 0;

    public static double MAX_ANGULAR_VELOCITY = 0;
    public static double MAX_ANGULAR_ACCELERATION = 0;

    public static Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);

    public static double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
    public static double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI;
  }
}
