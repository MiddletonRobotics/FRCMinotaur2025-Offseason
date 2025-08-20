package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalDeviceIO {
    
    @AutoLog
    public static class DigitalDeviceInputs {
        public boolean value = false;
    }
}
