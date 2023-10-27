package org.teamresistance.swerve_base;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private static final RobotType robot = RobotType.ROBOT_2023C;
  public static final Mode currentMode = Mode.REAL;
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;

  public static final Map<RobotType, String> logFolders =
      Map.of(RobotType.ROBOT_2023C, "/media/sda2/");

  public static RobotType getRobot() {
    return robot;
  }

  public static Mode getMode() {
    switch (getRobot()) {
      case ROBOT_2023C:
      case ROBOT_2023P:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case ROBOT_SIMBOT:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    ROBOT_2023C,
    ROBOT_2023P,
    ROBOT_SIMBOT
  }
}
