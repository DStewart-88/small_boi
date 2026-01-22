package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Tray;

/** Wires driver controls to subsystems/commands. */
public class RobotContainer {
  private static final double kDebounceSeconds = 0.10;

  private final Tray tray = new Tray();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    // Initialize dashboard button for testing powerMotorAtDutyCycle
    SmartDashboard.putBoolean("Tray/TestPowerMotor", false);
    
    configureBindings();
  }

  private void configureBindings() {
    // Debounced A-button triggers a single two-rotation tray move.
    driverController
        .a()
        .debounce(kDebounceSeconds)
        .onTrue(new InstantCommand(tray::rotateMotorTwoRotations, tray));
    
    // Dashboard momentary button for testing powerMotorAtDutyCycle
    new Trigger(() -> SmartDashboard.getBoolean("Tray/TestPowerMotor", false))
        .whileTrue(new RunCommand(tray::powerMotorAtDutyCycle, tray))
        .onFalse(new InstantCommand(tray::stopMotor, tray));
  }

  public Tray getTray() {
    return tray;
  }
}
