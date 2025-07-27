package frc.robot.utilities.constants;

import edu.wpi.first.wpilibj.RobotBase;

public class GlobalConstants {
    public static final boolean kTuningMode = true;

    public static final Mode kRealMode = Mode.REAL;
    public static final Mode kSimMode = Mode.SIM;
    public static final Mode kCurrentMode = RobotBase.isReal() ? kRealMode : kSimMode;

    public static final boolean kUsePhoenixDiagnosticServer = false;
    public static final boolean kUseBaseRefreshManager = false;

    public static final boolean kUseComponents = true;
    public static final boolean kUseAlerts = true && kCurrentMode != Mode.SIM;


    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,
    
        /** Replaying from a log file. */
        REPLAY
      }
    
}
