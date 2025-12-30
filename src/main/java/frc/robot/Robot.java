// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.LogServer;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
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
 * - Dashboard outputs every loop to SmartDashboard (Robot/* keys).
 * - Serves deploy directory for remote layout downloads (Elastic, etc.).
 */
public class Robot extends LoggedRobot {
  // --- Heartbeat/state ---
  private String m_currentMode = "Disabled"; // Updated in each *Init()
  private double m_lastHeartbeatTs = 0.0; // Seconds (FPGATimestamp)
  private int m_heartbeatCount = 0; // Increments once per second

  // --- Subsystems ---
  private RobotContainer robotContainer;
  
  // --- Disabled-only HTTP log server ---
  private static final int LOG_SERVER_PORT = 5801; // Keep 5800 free for deploy web server
  private LogServer m_logServer;
  private Thread m_logServerThread;

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

  /** Called when the robot first starts. */
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();

    // Serve deploy directory for remote layout downloading (Elastic) on port 5800.
    WebServer.start(5800, Filesystem.getDeployDirectory().getAbsolutePath());
  }

  /**
   * Runs every 20 ms in all modes. Performs heartbeat printing once per second and publishes
   * status to SmartDashboard.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

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

    // SmartDashboard (Robot/* keys) updated every loop.
    SmartDashboard.putString("Robot/Mode", m_currentMode);
    SmartDashboard.putNumber("Robot/Heartbeat", m_heartbeatCount);
    SmartDashboard.putBoolean("Robot/DSConnected", dsAttached);
    SmartDashboard.putBoolean("Robot/FMSAttached", fmsAttached);
    SmartDashboard.putNumber("Robot/BatteryVolts", batteryVolts);
    SmartDashboard.putBoolean("Robot/UserButton", userButton);
  }

  // --- Mode transitions: log and update current mode label ---
  @Override
  public void disabledInit() {
    m_currentMode = "Disabled";
    System.out.println(">>> Mode changed to: " + m_currentMode);
    startLogServerIfNeeded();
  }

  @Override
  public void autonomousInit() {
    m_currentMode = "Autonomous";
    System.out.println(">>> Mode changed to: " + m_currentMode);
    stopLogServer();
  }

  @Override
  public void teleopInit() {
    m_currentMode = "Teleop";
    System.out.println(">>> Mode changed to: " + m_currentMode);
    stopLogServer();
  }

  @Override
  public void testInit() {
    m_currentMode = "Test";
    System.out.println(">>> Mode changed to: " + m_currentMode);
    stopLogServer();
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

  // --- Log HTTP server helpers ---
  private Path selectLogDirForServer() {
    // Prefer USB if present; otherwise internal storage.
    try {
      Path usb = Paths.get("/U/logs");
      if (Files.isDirectory(usb)) {
        return usb;
      }
    } catch (Exception ignored) {}
    return Paths.get("/home/lvuser/logs");
  }

  private void startLogServerIfNeeded() {
    if (m_logServerThread != null && m_logServerThread.isAlive()) {
      return;
    }
    Path logDir = selectLogDirForServer();
    m_logServer = new LogServer(LOG_SERVER_PORT, logDir);
    m_logServerThread = new Thread(m_logServer, "LogServer-main");
    m_logServerThread.setDaemon(true);
    m_logServerThread.start();
  }

  private void stopLogServer() {
    LogServer srv = m_logServer;
    Thread t = m_logServerThread;
    m_logServer = null;
    m_logServerThread = null;
    if (srv != null) {
      try {
        srv.stop();
      } catch (Exception ignored) {}
    }
    if (t != null) {
      try {
        t.join(200);
      } catch (InterruptedException ignored) {}
    }
  }
}
