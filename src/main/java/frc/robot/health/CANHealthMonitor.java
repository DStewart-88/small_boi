package frc.robot.health;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized CAN health tracker that mirrors connectivity to Shuffleboard and AdvantageKit.
 *
 * <p>Usage: call {@link #updateStatus(String, boolean)} from subsystems with the device key and a
 * boolean indicating if that device is currently connected/healthy.
 */
public final class CANHealthMonitor {
  private static final CANHealthMonitor INSTANCE = new CANHealthMonitor();

  // Deterministic device ordering for the "CAN Status" tab.
  private static final Map<String, List<String>> wiringOrder = buildWiringOrder();

  private final Map<String, Boolean> lastState = new HashMap<>();
  private final Map<String, GenericEntry> shuffleEntries = new HashMap<>();
  private ShuffleboardTab canTab;

  private CANHealthMonitor() {}

  public static CANHealthMonitor getInstance() {
    return INSTANCE;
  }

  /** Creates the Shuffleboard tab and pre-populates known device keys. Safe to call once. */
  public void setupShuffleboard() {
    ensureTab();

    // Pre-create widgets for the known device set to keep layout stable.
    for (List<String> devices : wiringOrder.values()) {
      for (String key : devices) {
        ensureEntry(key);
      }
    }

    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Update the connectivity status for a CAN device.
   *
   * @param key Logical device key (e.g., "Drive/FrontLeft/DriveMotor").
   * @param connected True if the device is healthy/connected.
   */
  public void updateStatus(String key, boolean connected) {
    ensureTab();
    GenericEntry entry = ensureEntry(key);
    entry.setBoolean(connected);

    Logger.recordOutput("CANHealth/" + key, connected);

    Boolean previous = lastState.get(key);
    if (previous != null && previous != connected) {
      DriverStation.reportWarning(
          "[CANHealth] " + key + " changed to " + (connected ? "CONNECTED" : "DISCONNECTED"),
          false);
    }
    lastState.put(key, connected);
  }

  private void ensureTab() {
    if (canTab == null) {
      canTab = Shuffleboard.getTab("CAN Status");
    }
  }

  private GenericEntry ensureEntry(String key) {
    GenericEntry entry = shuffleEntries.get(key);
    if (entry == null) {
      entry =
          canTab
              .add(key, false)
              .withWidget(BuiltInWidgets.kBooleanBox)
              .getEntry();
      shuffleEntries.put(key, entry);
    }
    return entry;
  }

  private static Map<String, List<String>> buildWiringOrder() {
    Map<String, List<String>> order = new LinkedHashMap<>();
    order.put(
        "Rio",
        List.of(
            //"Drive/Gyro", // Bench pigeon on RIO bus
            "Tray/Motor",
            "Tray/CANcoder"));

    /*order.put(
        "CANivore",
        List.of(
            "Drive/FrontLeft/DriveMotor",
            "Drive/FrontLeft/TurnMotor",
            "Drive/FrontLeft/TurnEncoder",
            "Drive/FrontRight/DriveMotor",
            "Drive/FrontRight/TurnMotor",
            "Drive/FrontRight/TurnEncoder",
            "Drive/BackLeft/DriveMotor",
            "Drive/BackLeft/TurnMotor",
            "Drive/BackLeft/TurnEncoder",
            "Drive/BackRight/DriveMotor",
            "Drive/BackRight/TurnMotor",
            "Drive/BackRight/TurnEncoder"));
            */
    return Collections.unmodifiableMap(order);
  }
}
