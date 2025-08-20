package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface CANCoderIO {
    
    @AutoLog
    public static class CANCoderInputs {
        protected StatusCode status = StatusCode.OK;
        protected int faultField = 0;
        protected int stickyFaultField = 0;
        protected double position = 0.0;
        protected double absolutePosition = 0.0;
        protected double velocity = 0.0;
    }
}