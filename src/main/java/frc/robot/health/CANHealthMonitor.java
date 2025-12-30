package frc.robot.health;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized CAN health tracker that mirrors connectivity to NetworkTables and AdvantageKit.
 *
 * <p>Usage: call {@link #updateStatus(String, boolean)} from subsystems with the device key and a
 * boolean indicating if that device is currently connected/healthy.
 */
public final class CANHealthMonitor {
  private static final String NT_BASE_PATH = "/CANHealth/";
  private static final CANHealthMonitor INSTANCE = new CANHealthMonitor();

  private final Map<String, Boolean> lastState = new HashMap<>();
  private final Map<String, BooleanPublisher> publishers = new HashMap<>();

  private CANHealthMonitor() {}

  public static CANHealthMonitor getInstance() {
    return INSTANCE;
  }

  /**
   * Update the connectivity status for a CAN device.
   *
   * @param key Logical device key (e.g., "Drive/FrontLeft/DriveMotor").
   * @param connected True if the device is healthy/connected.
   */
  public void updateStatus(String key, boolean connected) {
    ensurePublisher(key).set(connected);
    Logger.recordOutput("CANHealth/" + key, connected);

    Boolean previous = lastState.get(key);
    if (previous != null && previous != connected) {
      DriverStation.reportWarning(
          "[CANHealth] " + key + " changed to " + (connected ? "CONNECTED" : "DISCONNECTED"),
          false);
    }
    lastState.put(key, connected);
  }

  private BooleanPublisher ensurePublisher(String key) {
    BooleanPublisher pub = publishers.get(key);
    if (pub == null) {
      String topic = NT_BASE_PATH + key;
      pub = NetworkTableInstance.getDefault().getBooleanTopic(topic).publish();
      publishers.put(key, pub);
    }
    return pub;
  }
}
