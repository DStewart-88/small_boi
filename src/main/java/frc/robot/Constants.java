package frc.robot;

/** Central home for robot-wide constants. */
public final class Constants {
  private Constants() {}

  public static final class TrayConstants {
    public static final int kMotorId = 2;
    public static final int kCancoderId = 1;

    public static final double kTwoRotations = 2.0;

    // Closed-loop gains for the simple position move.
    public static final double kPositionKp = 5.0;
    public static final double kPositionKi = 0.0;
    public static final double kPositionKd = 0.1;

    // Conservative supply current limit for bench use (raise as needed later).
    public static final double kSupplyCurrentLimitAmps = 5.0;
  }
}
