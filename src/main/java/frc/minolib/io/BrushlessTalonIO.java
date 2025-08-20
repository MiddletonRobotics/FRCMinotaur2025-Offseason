package frc.minolib.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;

public interface BrushlessTalonIO {

    @AutoLog
    public static class TalonFXInputs {
        public StatusCode status = StatusCode.OK;
        public int faultField = 0;
        public int stickyFaultField = 0;
        public double percentOutput = 0.0;
        public double supplyCurrent = 0.0;
        public double statorCurrent = 0.0;
        public double torqueCurrent = 0.0;
        public double closedLoopReference = 0.0;
        public double closedLoopReferenceSlope = 0.0;
        public double rotorPosition = 0.0;
        public double sensorPosition = 0.0;
        public double latencyCompensatedSensorPosition = 0.0;
        public double sensorVelocity = 0.0;
        public double temperature = 0.0;
    }
}