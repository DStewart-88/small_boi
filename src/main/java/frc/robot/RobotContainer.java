package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Tray;

/** Wires driver controls to subsystems/commands. */
public class RobotContainer {
  private static final double kDebounceSeconds = 0.10;

  private final Tray tray = new Tray();
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Debounced A-button triggers a single two-rotation tray move.
    driverController
        .a()
        .debounce(kDebounceSeconds)
        .onTrue(new InstantCommand(tray::rotateMotorTwoRotations, tray));
  }

  public Tray getTray() {
    return tray;
  }
}
