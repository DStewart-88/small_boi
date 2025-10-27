package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TestSubsystem: reads a single CTRE CANcoder and publishes absolute position and velocity
 * to Shuffleboard and SmartDashboard.
 *
 * - Device ID defaults to 3 (editable later).
 * - Status signals are configured to 10 Hz and refreshed in periodic().
 * - Read-only; no motor/control interactions.
 */
public class TestSubsystem {
  // --- Hardware ---
  private final CANcoder cancoder = new CANcoder(3);

  // --- Shuffleboard entries on the "Test" tab ---
  private final GenericEntry absEntry;
  private final GenericEntry velEntry;
  private final GenericEntry canStatusEntry;

  public TestSubsystem() {
    System.out.println("TestSubsystem initialized.");

    // Configure CAN status frequency for key signals to 10 Hz.
    BaseStatusSignal.setUpdateFrequencyForAll(10.0, cancoder.getPosition(), cancoder.getVelocity());

    // Create Shuffleboard tab and entries for live display.
    ShuffleboardTab tab = Shuffleboard.getTab("Test");
    absEntry = tab.add("AbsolutePosition", 0.0).getEntry();
    velEntry = tab.add("Velocity", 0.0).getEntry();
    canStatusEntry = tab.add("CANStatus", "Unknown").getEntry();
  }

  /**
   * Refreshes CANcoder signals and publishes values to Shuffleboard and SmartDashboard.
   * Call from Robot.robotPeriodic() to run continuously.
   */
  public void periodic() {
    // Refresh signals from the CAN bus in a single call.
    var pos = cancoder.getPosition();
    var vel = cancoder.getVelocity();
    // Capture CAN refresh status to report health/connectivity.
    StatusCode canStatus = BaseStatusSignal.refreshAll(pos, vel);

    // Read values (rotations and rotations/sec) as doubles.
    double absRot = pos.getValueAsDouble();
    double velRps = vel.getValueAsDouble();

    // SmartDashboard paths
    SmartDashboard.putNumber("Test/AbsolutePosition", absRot);
    SmartDashboard.putNumber("Test/Velocity", velRps);

    // Shuffleboard entries
    absEntry.setDouble(absRot);
    velEntry.setDouble(velRps);
    canStatusEntry.setString(canStatus.toString());

    // SmartDashboard CAN status (string and boolean OK flag)
    SmartDashboard.putString("Test/CANStatus", canStatus.toString());
    SmartDashboard.putBoolean("Test/CANOK", canStatus == StatusCode.OK);
  }
}
