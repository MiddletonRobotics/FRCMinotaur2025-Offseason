package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GyroInputs {
    public boolean isGyroConnected = false;
    public int faultField = 0;
    public int stickyFaultField = 0;
    public double roll = 0.0;
    public double pitch = 0.0;
    public double yaw = 0.0;
    public double latencyCompensatedYaw = 0.0;
    public double rollRate = 0.0;
    public double pitchRate = 0.0;
    public double yawRate = 0.0;
}
