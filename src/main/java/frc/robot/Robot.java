// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.TestSubsystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Minimal production-shaped TimedRobot skeleton.
 *
 * Functional blocks:
 * - Heartbeat once per second from robotPeriodic() with current mode, DS/FMS attached, battery
 *   voltage, and RIO User Button state. No blocking sleeps; uses WPILib timers.
 * - Mode transition logging on each *Init(): prints ">>> Mode changed to: <Mode>".
 * - Dashboard outputs every loop to SmartDashboard (Robot/* keys) and a Shuffleboard "Robot" tab.
 * - NetworkTables is flushed once in robotInit() after widget setup so the tab appears quickly.
 */
public class Robot extends LoggedRobot {
  // --- Heartbeat/state ---
  private String m_currentMode = "Disabled"; // Updated in each *Init()
  private double m_lastHeartbeatTs = 0.0; // Seconds (FPGATimestamp)
  private int m_heartbeatCount = 0; // Increments once per second

  // --- Subsystems ---
  private final TestSubsystem testSubsystem = new TestSubsystem();

  // --- Shuffleboard entries on the "Robot" tab ---
  private GenericEntry m_modeEntry;
  private GenericEntry m_heartbeatEntry;
  private GenericEntry m_dsEntry;
  private GenericEntry m_fmsEntry;
  private GenericEntry m_voltsEntry;
  private GenericEntry m_userButtonEntry;

  public Robot() {
    // AdvantageKit logger setup
    Logger.recordMetadata("ProjectName", "RIO2-Testbed");

    if (isReal()) {
      // Force logging to USB at /U/logs with internal fallback.
      String usbLogPath = "/U/logs";
      String internalLogPath = "/home/lvuser/logs";
      boolean usingUsb = false;

      try {
        Path usbPath = Paths.get(usbLogPath);
        Files.createDirectories(usbPath);
        if (Files.isDirectory(usbPath) && Files.isWritable(usbPath)) {
          Logger.addDataReceiver(new WPILOGWriter(usbLogPath));
          Logger.recordMetadata("LogStorage", usbLogPath);
          usingUsb = true;
        }
      } catch (Exception e) {
        // Will fall back to internal storage below.
      }

      if (!usingUsb) {
        try {
          Path internalPath = Paths.get(internalLogPath);
          Files.createDirectories(internalPath);
        } catch (Exception ignored) {
        }
        Logger.addDataReceiver(new WPILOGWriter(internalLogPath));
        Logger.recordMetadata("LogStorage", internalLogPath);
      }

      Logger.addDataReceiver(new NT4Publisher());
    } else {
      setUseTiming(false); // Fast replay in sim
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    Logger.start();
  }

  /** Called when the robot first starts. Sets up dashboard widgets and flushes NT once. */
  @Override
  public void robotInit() {
    // Initialize Shuffleboard widgets on a dedicated "Robot" tab.
    ShuffleboardTab tab = Shuffleboard.getTab("Robot");
    m_modeEntry = tab.add("Mode", m_currentMode).getEntry();
    m_heartbeatEntry = tab.add("Heartbeat", m_heartbeatCount).getEntry();
    m_dsEntry = tab.add("DSConnected", false).getEntry();
    m_fmsEntry = tab.add("FMSAttached", false).getEntry();
    m_voltsEntry = tab.add("BatteryVolts", 0.0).getEntry();
    m_userButtonEntry = tab.add("UserButton", false).getEntry();

    // Flush once so the tab and widgets appear quickly in Shuffleboard.
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Runs every 20 ms in all modes. Performs heartbeat printing once per second and publishes
   * status to SmartDashboard and Shuffleboard.
   */
  @Override
  public void robotPeriodic() {
    // Gather current status.
    boolean dsAttached = DriverStation.isDSAttached();
    boolean fmsAttached = DriverStation.isFMSAttached();
    double batteryVolts = RobotController.getBatteryVoltage();
    boolean userButton = RobotController.getUserButton();

    double now = Timer.getFPGATimestamp();
    if (now - m_lastHeartbeatTs >= 1.0) {
      m_lastHeartbeatTs = now;
      m_heartbeatCount++;

      // Console heartbeat line (once per second).
      System.out.printf(
          "Heartbeat #%d | Mode=%s | DS=%b | FMS=%b | Volts=%.2f | UserButton=%b%n",
          m_heartbeatCount, m_currentMode, dsAttached, fmsAttached, batteryVolts, userButton);
    }

    // SmartDashboard (Robot/* keys) — updated every loop.
    SmartDashboard.putString("Robot/Mode", m_currentMode);
    SmartDashboard.putNumber("Robot/Heartbeat", m_heartbeatCount);
    SmartDashboard.putBoolean("Robot/DSConnected", dsAttached);
    SmartDashboard.putBoolean("Robot/FMSAttached", fmsAttached);
    SmartDashboard.putNumber("Robot/BatteryVolts", batteryVolts);
    SmartDashboard.putBoolean("Robot/UserButton", userButton);

    // Shuffleboard entries — mirror the same values every loop.
    if (m_modeEntry != null) m_modeEntry.setString(m_currentMode);
    if (m_heartbeatEntry != null) m_heartbeatEntry.setDouble(m_heartbeatCount);
    if (m_dsEntry != null) m_dsEntry.setBoolean(dsAttached);
    if (m_fmsEntry != null) m_fmsEntry.setBoolean(fmsAttached);
    if (m_voltsEntry != null) m_voltsEntry.setDouble(batteryVolts);
    if (m_userButtonEntry != null) m_userButtonEntry.setBoolean(userButton);

    // Update the TestSubsystem (CANcoder readout) alongside the heartbeat.
    testSubsystem.periodic();
  }

  // --- Mode transitions: log and update current mode label ---
  @Override
  public void disabledInit() {
    m_currentMode = "Disabled";
    System.out.println(">>> Mode changed to: " + m_currentMode);
  }

  @Override
  public void autonomousInit() {
    m_currentMode = "Autonomous";
    System.out.println(">>> Mode changed to: " + m_currentMode);
  }

  @Override
  public void teleopInit() {
    m_currentMode = "Teleop";
    System.out.println(">>> Mode changed to: " + m_currentMode);
  }

  @Override
  public void testInit() {
    m_currentMode = "Test";
    System.out.println(">>> Mode changed to: " + m_currentMode);
  }

  // Periodic stubs for each mode (kept minimal; shared work is in robotPeriodic()).
  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testPeriodic() {}

  // Simulation stubs (left as-is for completeness).
  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
