package frc.constants;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterConstants {
  public static final double INTAKE_HEIGHT = 0;
  public static final Rotation2d INTAKE_ANGLE = Rotation2d.fromDegrees(231);

  public static final class TiltConstants {
    public static final double GRAVITY_GAIN = 0;
    public static final double STATIC_GAIN = 0;
    public static final double VELOCITY_GAIN = 0;

    public static final double P_GAIN = 6;
    public static final double I_GAIN = 0;
    public static final double D_GAIN = 0;

    public static final double MAX_ANGULAR_VELOCITY = 1.0;
    public static final double MAX_ANGULAR_ACCELERATION = 2.0;

    public static final Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(3);

    public static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;
    public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / 60;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final int CURRENT_LIMIT = 50;
  }

  public static final class HeightConstants {
    public static final double GRAVITY_GAIN = 0;
    public static final double STATIC_GAIN = 0;
    public static final double VELOCITY_GAIN = 0;

    public static final double P_GAIN = 200;
    public static final double I_GAIN = 0;
    public static final double D_GAIN = 0;

    public static final double MAX_VELOCITY = 0.05;
    public static final double MAX_ACCELERATION = 0.5;

    public static final double TOLERANCE = 0.03;

    public static final double POSITION_CONVERSION_FACTOR = 0.006;
    public static final double VELOCITY_CONVERSION_FACTOR = 0.0001;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;
    public static final int CURRENT_LIMIT = 50;
  }

  public static class ShooterMotorConstants {
    public static int CURRENT_LIMIT = 50;
    public static IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final double TOLERANCE = 0.5;

    public static final double POSITION_CONVERSION_FACTOR = Math.PI * 0.0762;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;

    public static final double P_GAIN = 0;
    public static final double I_GAIN = 0;
    public static final double D_GAIN = 0;

    public static final double STATIC_GAIN = 0;
    public static final double VELOCITY_GAIN = 0.6;
  }

  public static final class IntermediateConstants {
    public static final int CURRENT_LIMIT = 20;
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final double SHOOT_SPEED = 1;
    public static final double SHOOT_TIME = 1;

    public static final double P_GAIN = 25;
    public static final double I_GAIN = 0;
    public static final double D_GAIN = 0;

    public static final double TOLERANCE = 0.02;

    public static final double POSITION_CONVERSION_FACTOR = (1.0 / 15.0) * Math.PI * 0.0508;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;

    public static final double STATIC_GAIN = 1;
    public static final double VELOCITY_GAIN = 0.3;

    public static final double MAX_VELOCITY = 0.5;
    public static final double MAX_ACCELERATION = 1;

    public static final double FINAL_OFFSET = 0.35;
  }

  public static final Rotation2d AMP_ANGLE = Rotation2d.fromDegrees(140);
  public static final double AMP_HEIGHT = 0.285;
  public static final double AMP_SPEED = 7;
  public static final double SPEAKER_SCORE_HEIGHT = 0.1;
}
