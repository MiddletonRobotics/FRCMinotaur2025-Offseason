package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface CANCoderIO {
    
    @AutoLog
    public static class CANCoderInputs {
        public StatusCode status = StatusCode.OK;
        public int faultField = 0;
        public int stickyFaultField = 0;
        public double position = 0.0;
        public double absolutePosition = 0.0;
        public double velocity = 0.0;
    }
}