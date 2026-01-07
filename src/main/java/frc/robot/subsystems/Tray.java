package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrayConstants;
import frc.robot.health.CANHealthMonitor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Tray subsystem: single Falcon 500 (TalonFX) with an external CANcoder.
 *
 * <p>- CANcoder publishes absolute output shaft position for logging/observation.
 * <p>- TalonFX uses its integrated encoder for motion control (repeatable, low-latency sensor path).
 * <p>- Provides a simple 2-rotation closed-loop move for on-robot testing.
 */
public class Tray extends SubsystemBase {
  private static final double DEGREES_PER_ROTATION = 360.0;

  // --- Hardware ---
  private final TalonFX trayMotor = new TalonFX(TrayConstants.kMotorId);
  private final CANcoder trayCancoder = new CANcoder(TrayConstants.kCancoderId);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

  private double lastCommandedTargetRotations = 0.0;

  public Tray() {
    System.out.println("Tray subsystem initialized.");
    configureMotor();

    // Refresh the signals we care about at 50 Hz so logs/dashboard track motion cleanly.
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        trayMotor.getPosition(),
        trayMotor.getVelocity(),
        trayMotor.getSupplyCurrent(),
        trayMotor.getSupplyVoltage(),
        trayCancoder.getAbsolutePosition());
    trayMotor.optimizeBusUtilization();
    trayCancoder.optimizeBusUtilization();
  }

  private void configureMotor() {
    // Keep motor config local to the subsystem so test moves are predictable.
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Basic closed-loop gains for position holding (tune as needed on-robot).
    cfg.Slot0.kP = TrayConstants.kPositionKp;
    cfg.Slot0.kI = TrayConstants.kPositionKi;
    cfg.Slot0.kD = TrayConstants.kPositionKd;
    cfg.Slot0.kV = 0.0;

    // Conservative supply current limit for bench bring-up (easy to adjust in constants).
    cfg.CurrentLimits.SupplyCurrentLimit = TrayConstants.kSupplyCurrentLimitAmps;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = true;

    trayMotor.getConfigurator().apply(cfg);

    // Zero the integrated sensor so 2-rotation moves are repeatable from boot.
    trayMotor.setPosition(0.0);
  }

  /**
   * Refreshes motor/CANcoder signals and publishes values to SmartDashboard.
   * Called by the command scheduler automatically.
   */
  @Override
  public void periodic() {
    var rotorPos = trayMotor.getPosition();
    var rotorVel = trayMotor.getVelocity();
    var supplyCurrent = trayMotor.getSupplyCurrent();
    var supplyVoltage = trayMotor.getSupplyVoltage();
    var absPos = trayCancoder.getAbsolutePosition();

    // Refresh all signals.
    BaseStatusSignal.refreshAll(rotorPos, rotorVel, supplyCurrent, supplyVoltage, absPos);

    // Read values (rotations and rotations/sec) as doubles.
    double rotorRotations = rotorPos.getValueAsDouble();
    double rotorVelRps = rotorVel.getValueAsDouble();

    // CANcoder absolute position comes back in rotations; convert to degrees for readability.
    double absDegrees = wrapCancoderDegrees(absPos.getValueAsDouble() * DEGREES_PER_ROTATION);

    double supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    double supplyVoltageVolts = supplyVoltage.getValueAsDouble();
    double powerWatts = supplyCurrentAmps * supplyVoltageVolts;

    // Publish CAN health for the tray motor and CANcoder.
    CANHealthMonitor.getInstance()
        .updateStatus("Tray/Motor", trayMotor.isConnected());
    CANHealthMonitor.getInstance()
        .updateStatus("Tray/CANcoder", trayCancoder.isConnected());

    // SmartDashboard paths
    SmartDashboard.putNumber("Tray/MotorRotations", rotorRotations);
    SmartDashboard.putNumber("Tray/MotorRPS", rotorVelRps);
    SmartDashboard.putNumber("Tray/AbsoluteDegrees", absDegrees);
    SmartDashboard.putNumber("Tray/TargetRotations", lastCommandedTargetRotations);
    SmartDashboard.putNumber("Tray/SupplyCurrentAmps", supplyCurrentAmps);
    SmartDashboard.putNumber("Tray/SupplyVoltageVolts", supplyVoltageVolts);
    SmartDashboard.putNumber("Tray/PowerWatts", powerWatts);
    SmartDashboard.putBoolean("Tray/CANOK", trayMotor.isConnected() && trayCancoder.isConnected());

    // AdvantageKit outputs (recorded every cycle)
    Logger.recordOutput("Tray/MotorRotations", rotorRotations);
    Logger.recordOutput("Tray/MotorDegrees", rotorRotations * DEGREES_PER_ROTATION);
    Logger.recordOutput("Tray/TargetRotations", lastCommandedTargetRotations);
    Logger.recordOutput("Tray/CANcoderDegrees", absDegrees);
    Logger.recordOutput("Tray/SupplyCurrentAmps", supplyCurrentAmps);
    Logger.recordOutput("Tray/SupplyVoltageVolts", supplyVoltageVolts);
    Logger.recordOutput("Tray/PowerWatts", powerWatts);
    Logger.recordOutput("Tray/CANOK", trayMotor.isConnected() && trayCancoder.isConnected());
  }

  /**
   * Command the Falcon to move exactly +2 motor rotations using its integrated encoder.
   * TalonFX position units are already in rotations, so we convert desired rotations -> control
   * units explicitly by adding the delta to the current position reading.
   */
  public void rotateMotorTwoRotations() {
    double currentRotations = trayMotor.getPosition().getValueAsDouble();
    lastCommandedTargetRotations = currentRotations + TrayConstants.kTwoRotations;

    // Closed-loop position request; holds after it reaches the target.
    trayMotor.setControl(positionRequest.withPosition(lastCommandedTargetRotations));
  }

  /** Absolute CANcoder position in degrees (0-360 wrap). */
  public double getAbsoluteTrayPosition() {
    double rawDegrees = trayCancoder.getAbsolutePosition().getValueAsDouble() * DEGREES_PER_ROTATION;
    return wrapCancoderDegrees(rawDegrees);
  }

  // Phoenix absolute position reports [-180, 180) deg; wrap to [0, 360) for dashboards/logs.
  private double wrapCancoderDegrees(double degrees) {
    double wrapped = degrees % DEGREES_PER_ROTATION;
    return wrapped < 0 ? wrapped + DEGREES_PER_ROTATION : wrapped;
  }

  // AdvantageKit autolog outputs (recorded every cycle)
  @AutoLogOutput(key = "Tray/AbsDegrees")
  public double getAbsDegrees() {
    return getAbsoluteTrayPosition();
  }

  @AutoLogOutput(key = "Tray/CANOK")
  public boolean isCANOK() {
    return trayMotor.isConnected() && trayCancoder.isConnected();
  }
}
