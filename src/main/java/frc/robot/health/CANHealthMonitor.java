package frc.robot.health;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized CAN health tracker that mirrors connectivity to NetworkTables and AdvantageKit.
 *
 * <p>Usage: call {@link #updateStatus(String, boolean)} from subsystems with the device key and a
 * boolean indicating if that device is currently connected/healthy.
 */
public final class CANHealthMonitor {
  private static final String NT_BASE_PATH = "/CANHealth/";
  private static final String RESET_KEY = "CANHealth/ResetStickyFaults";
  private static final double SUMMARY_PERIOD_SEC = 0.5;
  private static final CANHealthMonitor INSTANCE = new CANHealthMonitor();

  private final Map<String, Boolean> lastState = new HashMap<>();
  private final Map<String, BooleanPublisher> publishers = new HashMap<>();
  private final Map<String, Boolean> currentStatus = new HashMap<>();
  private final Set<String> everDisconnected = new LinkedHashSet<>();

  private BooleanPublisher allConnectedPub;
  private StringPublisher currentFaultsPub;
  private StringPublisher stickyFaultsPub;
  private DoublePublisher resetCountPub;
  private DoublePublisher resetTimePub;
  private BooleanEntry resetEntry;

  private boolean stickyDirty = false;
  private double lastSummaryPublishSec = 0.0;
  private boolean lastResetRequest = false;
  private int stickyResetCount = 0;

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

    currentStatus.put(key, connected);
    if (!connected && everDisconnected.add(key)) {
      stickyDirty = true;
    }

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

  /** Publishes summary statuses on a throttled cadence and when sticky faults grow. */
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    processResetRequest(now);

    if (stickyDirty) {
      publishStickyFaults();
      stickyDirty = false;
      lastSummaryPublishSec = now; // avoid immediate heartbeat duplicate
    }

    if (now - lastSummaryPublishSec >= SUMMARY_PERIOD_SEC) {
      publishAllConnected();
      publishCurrentFaults();
      publishStickyFaults();
      lastSummaryPublishSec = now;
    }
  }

  private void publishAllConnected() {
    boolean allConnected = allConnectedNow();
    Logger.recordOutput("CANHealth/AllConnectedNow", allConnected);
    ensureAllConnectedPublisher().set(allConnected);
  }

  private void publishCurrentFaults() {
    String faults = currentFaultsString();
    Logger.recordOutput("CANHealth/CurrentFaults", faults);
    ensureCurrentFaultsPublisher().set(faults);
  }

  private void publishStickyFaults() {
    String sticky = stickyFaultsString();
    Logger.recordOutput("CANHealth/StickyFaults", sticky);
    ensureStickyFaultsPublisher().set(sticky);
  }

  private boolean allConnectedNow() {
    for (boolean ok : currentStatus.values()) {
      if (!ok) {
        return false;
      }
    }
    return true; // Empty means nothing is faulted yet.
  }

  private String currentFaultsString() {
    List<String> faults = new ArrayList<>();
    for (Map.Entry<String, Boolean> entry : currentStatus.entrySet()) {
      if (!Boolean.TRUE.equals(entry.getValue())) {
        faults.add(entry.getKey());
      }
    }
    if (faults.isEmpty()) {
      return "OK";
    }
    Collections.sort(faults);
    return String.join("; ", faults);
  }

  private String stickyFaultsString() {
    if (everDisconnected.isEmpty()) {
      return "OK";
    }
    return String.join("; ", everDisconnected);
  }

  /** @return true if any known device is currently disconnected. */
  public boolean hasAnyCurrentFault() {
    return !allConnectedNow();
  }

  /** @return true if any device has ever been seen disconnected since boot. */
  public boolean hasAnyStickyFault() {
    return !everDisconnected.isEmpty();
  }

  /** @return true if all known devices are currently connected (or none reported yet). */
  public boolean isAllConnectedNow() {
    return allConnectedNow();
  }

  private BooleanPublisher ensureAllConnectedPublisher() {
    if (allConnectedPub == null) {
      allConnectedPub =
          NetworkTableInstance.getDefault()
              .getBooleanTopic(NT_BASE_PATH + "AllConnectedNow")
              .publish();
    }
    return allConnectedPub;
  }

  private StringPublisher ensureCurrentFaultsPublisher() {
    if (currentFaultsPub == null) {
      currentFaultsPub =
          NetworkTableInstance.getDefault()
              .getStringTopic(NT_BASE_PATH + "CurrentFaults")
              .publish();
    }
    return currentFaultsPub;
  }

  private StringPublisher ensureStickyFaultsPublisher() {
    if (stickyFaultsPub == null) {
      stickyFaultsPub =
          NetworkTableInstance.getDefault()
              .getStringTopic(NT_BASE_PATH + "StickyFaults")
              .publish();
    }
    return stickyFaultsPub;
  }

  private DoublePublisher ensureResetCountPublisher() {
    if (resetCountPub == null) {
      resetCountPub =
          NetworkTableInstance.getDefault()
              .getDoubleTopic(NT_BASE_PATH + "StickyFaultsResetCount")
              .publish();
    }
    return resetCountPub;
  }

  private DoublePublisher ensureResetTimePublisher() {
    if (resetTimePub == null) {
      resetTimePub =
          NetworkTableInstance.getDefault()
              .getDoubleTopic(NT_BASE_PATH + "StickyFaultsLastResetTimeSec")
              .publish();
    }
    return resetTimePub;
  }

  private BooleanEntry ensureResetEntry() {
    if (resetEntry == null) {
      resetEntry =
          NetworkTableInstance.getDefault()
              .getBooleanTopic(RESET_KEY)
              .getEntry(false);
      resetEntry.set(false);
      lastResetRequest = false;
    }
    return resetEntry;
  }

  private void processResetRequest(double now) {
    boolean resetRequest = ensureResetEntry().get(false);
    boolean risingEdge = resetRequest && !lastResetRequest;

    if (risingEdge) {
      if (DriverStation.isDisabled()) {
        everDisconnected.clear();
        stickyDirty = true;
        stickyResetCount++;
        ensureResetCountPublisher().set(stickyResetCount);
        Logger.recordOutput("CANHealth/StickyFaultsResetCount", stickyResetCount);
        ensureResetTimePublisher().set(now);
      }
      // Acknowledge by resetting the NT flag regardless of enabled state.
      ensureResetEntry().set(false);
      lastResetRequest = false;
    } else {
      lastResetRequest = resetRequest;
    }
  }
}
