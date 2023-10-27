package org.teamresistance.swerve_base.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import org.teamresistance.swerve_base.Constants;
import org.teamresistance.swerve_base.util.NavX;

public class GyroIONavX implements GyroIO {
  public static NavX navX;

  private static Rotation3d rotation;

  public GyroIONavX() {
    System.out.println("[Init] Creating NavX");

    switch (Constants.getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        navX = new NavX(SPI.Port.kMXP);
        break;
      default:
        throw new RuntimeException("Invalid robot for GyroIOPigeon2");
    }

    navX.reset();
    navX.resetDisplacement();
    navX.zeroYaw();
  }

  public void updateInputs(GyroIOInputs inputs) {
    navX.getRawGyroY();
    rotation = navX.getRotation3d();
    inputs.connected = navX.isConnected();
    inputs.rollPositionRad = Units.degreesToRadians(rotation.getX());
    inputs.pitchPositionRad = Units.degreesToRadians(rotation.getY());
    inputs.yawPositionRad = Units.degreesToRadians(rotation.getZ());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroX());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroY());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(navX.getRawGyroZ());
  }
}
