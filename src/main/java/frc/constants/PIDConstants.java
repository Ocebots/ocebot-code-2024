package frc.constants;

public class PIDConstants {
  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT =
      ModuleConstants.TURNING_ENCODER_POSITION_FACTOR; // radians
  public static final double DRIVING_P = 0.04;
  public static final double DRIVING_I = 0;
  public static final double DRIVING_D = 0;
  public static final double DRIVING_FF = 1 / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_RPS;
  public static final double DRIVING_MIN_OUTPUT = -1;
  public static final double DRIVING_MAX_OUTPUT = 1;
  public static final double TURNING_P = 1;
  public static final double TURNING_I = 0;
  public static final double TURNING_D = 0;
  public static final double TURNING_FF = 0;
  public static final double TURNING_MIN_OUTPUT = -1;
  public static final double TURNING_MAX_OUTPUT = 1;
}
