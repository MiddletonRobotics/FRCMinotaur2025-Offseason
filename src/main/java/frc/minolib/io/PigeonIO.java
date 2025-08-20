package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface PigeonIO {
    
    @AutoLog
    public static class Pigeon2Inputs {
        protected StatusCode status = StatusCode.OK;
        protected int faultField = 0;
        protected int stickyFaultField = 0;
        protected double roll = 0.0;
        protected double pitch = 0.0;
        protected double yaw = 0.0;
        protected double latencyCompensatedYaw = 0.0;
        protected double rollRate = 0.0;
        protected double pitchRate = 0.0;
        protected double yawRate = 0.0;
    }
}
