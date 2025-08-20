package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

public interface DigitalIO {
    
    @AutoLog
    public static class DigitalIOInputs {
        protected boolean value = false;
    }
}
