// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.teamresistance.swerve_base;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.teamresistance.swerve_base.commands.DriveWithJoysticks;
import org.teamresistance.swerve_base.subsystems.drive.*;
import org.teamresistance.swerve_base.util.Alert;
import org.teamresistance.swerve_base.util.AllianceFlipUtil;
import org.teamresistance.swerve_base.util.OverrideSwitches;
import org.teamresistance.swerve_base.util.SparkMaxBurnManager;

import java.util.function.Function;

public class RobotContainer {

  // OI objects
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);
  private final Trigger robotRelative = overrides.driverSwitch(0);
  private final Trigger manualDrive = overrides.operatorSwitch(0);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", Alert.AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", Alert.AlertType.WARNING);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", Alert.AlertType.INFO);
  private final Alert demoControlsActivated =
      new Alert("Demo controls are active. Do not use in competition.", Alert.AlertType.INFO);
  private final Alert demoSpeedActivated =
      new Alert("Demo speed limits are active. Do not use in competition.", Alert.AlertType.INFO);
  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 30.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 15.0);
  private final LoggedDashboardBoolean demoControls =
      new LoggedDashboardBoolean("Demo Controls", false);
  // Auto selector
  private final AutoSelector autoSelector = new AutoSelector("Auto");
  // Subsystems
  private Drive drive;
  private boolean lastWasDemoControls = false;

  public RobotContainer() {
    // Check if flash should be burned
    SparkMaxBurnManager.update();

    // Instantiate active subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2023C, ROBOT_2023P:
          drive =
              new Drive(
                new GyroIONavX(),
                  new ModuleIOSparkMax(0),
                  new ModuleIOSparkMax(1),
                  new ModuleIOSparkMax(2),
                  new ModuleIOSparkMax(3));

          break;
        case ROBOT_SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim(),
                  new ModuleIOSim());
          break;
      }
    }

    // Instantiate missing subsystems
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    // Startup alerts
    if (Constants.tuningMode) {
      new Alert("Tuning mode active, do not use in competition.", Alert.AlertType.INFO).set(true);
    }
    if (FieldConstants.isWPIField) {
      new Alert("WPI field selected, do not use in competition.", Alert.AlertType.INFO).set(true);
    }

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(
                    () -> {
                      driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                      operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                    })
                .withTimeout(1.5)
                .andThen(
                    Commands.run(
                            () -> {
                              driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                              operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                            })
                        .withTimeout(1.0)));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(1.0)));

    // Bind driver and operator controls
    System.out.println("[Init] Binding controls");
    bindControls(demoControls.get());
    lastWasDemoControls = demoControls.get();
    demoControlsActivated.set(demoControls.get());

    // Rely on our custom alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** Binds controls based on whether demo controls are active and update alerts. */
  public void updateDemoControls() {
    // Update control binding
    if (demoControls.get() != lastWasDemoControls) {
      bindControls(demoControls.get());
      lastWasDemoControls = demoControls.get();
    }

    // Update alerts
    demoControlsActivated.set(demoControls.get());
    demoSpeedActivated.set(DriveWithJoysticks.isDemo());
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
    operatorDisconnected.set(
        !demoControls.get()
            && (!DriverStation.isJoystickConnected(operator.getHID().getPort())
                || !DriverStation.getJoystickIsXbox(operator.getHID().getPort())));
    overrideDisconnected.set(!overrides.isConnected());
  }

  /** Binds the driver and operator controls. */
  public void bindControls(boolean demo) {
    // Clear old buttons
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Joystick command factories
    Function<Boolean, DriveWithJoysticks> driveWithJoysticksFactory =
        (Boolean sniper) ->
            new DriveWithJoysticks(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> sniper,
              robotRelative::getAsBoolean);

    // *** DRIVER CONTROLS ***

    // Drive controls
    final boolean[] demoManualArmMode = {false};
    Command driveWithJoysticksDefault =
      Commands.either(Commands.none(), driveWithJoysticksFactory.apply(false), () -> false)
            .withName("DriveWithJoysticks");
    Command oldDefaultCommand = CommandScheduler.getInstance().getDefaultCommand(drive);
    if (oldDefaultCommand != null) {
      oldDefaultCommand.cancel();
    }
    drive.setDefaultCommand(driveWithJoysticksDefault);
    driver
        .start()
        .or(driver.back())
        .and(() -> !demo || DriverStation.isDisabled())
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(
                          new Pose2d(
                              drive.getPose().getTranslation(),
                              AllianceFlipUtil.apply(new Rotation2d())));
                    })
                .ignoringDisable(true));
    if (!demo) {
      driver
          .b()
          .and(DriverStation::isDisabled)
          .whileTrue(
              Commands.run(
                      () -> {
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation().getX()
                                    + (driver.getLeftX() * Constants.loopPeriodSecs * 2.0),
                                drive.getPose().getTranslation().getY()
                                    - (driver.getLeftY() * Constants.loopPeriodSecs * 2.0),
                                drive
                                    .getRotation()
                                    .plus(
                                        new Rotation2d(
                                            -driver.getRightX()
                                                * Constants.loopPeriodSecs
                                                * 2.0))));
                      })
                  .ignoringDisable(true));
      driver.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    }

    // Log marker
    if (!demo) {
      driver
          .y()
          .whileTrue(
              Commands.startEnd(
                  () -> Logger.recordOutput("LogMarker", true),
                  () -> Logger.recordOutput("LogMarker", false))
                  .ignoringDisable(true));
    }
  }

  /** Passes the autonomous command to the {@link Robot} class. */
  public Command getAutonomousCommand() {
    return autoSelector.getCommand();
  }
}
