package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class GlobalConstants {
  public static final double kLowBatteryVoltage = 10.0;
  public static final double kLowBatteryDisabledTime = 1.5;
  public static final double kCANErrorTimeThreshold = 0.5; // Seconds to disable alert
  public static final double kCANivoreTimeThreshold = 0.5;

  public static final boolean isReplay = false;
  public static final boolean resimWithTiming = false;
  public static final boolean simLocalization = true;

  public static final String kCanivoreName = "*";

  public static final boolean kTuningMode = false;
  public static final boolean kDemoMode = false;
  public static final boolean kUseMapleSim = true;

  public static final Mode kRealMode = Mode.REAL;
  public static final Mode kSimMode = Mode.SIM;
  public static final Mode kCurrentMode = RobotBase.isReal() ? kRealMode : kSimMode;

  public static final boolean kUsePhoenixDiagnosticServer = false;
  public static final boolean kUseBaseRefreshManager = false;

  public static final boolean kUseComponents = true;
  public static final boolean kUseAlerts = true && kCurrentMode != Mode.SIM;

  public static final double kLoopPeriodSeconds = 0.02;

  public static final double kControllerDeadband = 0.08;


  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean isBlueAlliance() {
      return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
  }
}