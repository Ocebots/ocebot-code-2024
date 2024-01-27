package frc.constants;

import com.revrobotics.CANSparkBase.IdleMode;
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

    public static IdleMode IDLE_MODE = IdleMode.kBrake;
    public static int CURRENT_LIMIT = 50;
  }

  public static class HeightConstants {
    public static double GRAVITY_GAIN = 0;
    public static double STATIC_GAIN = 0;
    public static double VELOCITY_GAIN = 0;

    public static double P_GAIN = 0;
    public static double I_GAIN = 0;
    public static double D_GAIN = 0;

    public static double MAX_VELOCITY = 0;
    public static double MAX_ACCELERATION = 0;

    public static double TOLERANCE = 0.03;

    public static double POSITION_CONVERSION_FACTOR = 0;
    public static double VELOCITY_CONVERSION_FACTOR = 0;

    public static IdleMode IDLE_MODE = IdleMode.kBrake;
    public static int CURRENT_LIMIT = 50;
  }

  public static class ShooterMotorConstants {
    public static int CURRENT_LIMIT = 50;
    public static IdleMode IDLE_MODE = IdleMode.kBrake;

    public static double TOLERANCE = 0.5;

    public static double POSITION_CONVERSION_FACTOR = 0;
    public static double VELOCITY_CONVERSION_FACTOR = 0;

    public static double P_GAIN = 0;
    public static double I_GAIN = 0;
    public static double D_GAIN = 0;

    public static double STATIC_GAIN = 0;
    public static double VELOCITY_GAIN = 0;
  }

  public static class IntermediateConstants {
    public static int CURRENT_LIMIT = 20;
    public static IdleMode IDLE_MODE = IdleMode.kBrake;

    public static double SHOOT_SPEED = 0.5;
    public static double SHOOT_TIME = 1;
  }
}
