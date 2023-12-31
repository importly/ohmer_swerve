// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.teamresistance.swerve_base.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  /**
   * Updates the set of loggable inputs.
   */
  default void updateInputs(ModuleIOInputs inputs) {
  }

  /**
   * Run the drive motor at the specified voltage.
   */
  default void setDriveVoltage(double volts) {
  }

  /**
   * Run the turn motor at the specified voltage.
   */
  default void setTurnVoltage(double volts) {
  }

  /**
   * Enable or disable brake mode on the drive motor.
   */
  default void setDriveBrakeMode(boolean enable) {
  }

  /**
   * Enable or disable brake mode on the turn motor.
   */
  default void setTurnBrakeMode(boolean enable) {
  }

  @AutoLog
  class ModuleIOInputs {
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};
    public double[] driveTempCelcius = new double[] {};

    public double turnAbsolutePositionRad = 0.0;
    public double turnPositionRad = 0.0;
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};
    public double[] turnTempCelcius = new double[] {};
  }
}
