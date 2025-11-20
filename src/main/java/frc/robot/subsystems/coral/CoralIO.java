package frc.robot.subsystems.coral;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface CoralIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class CoralIOInputs {
        public boolean firstCANRangeTripped = false;
        public double firstCANRangeSignalStrength = 0.0;
        public double firstCANRangeDistanceInMeters = 0.0;
        public boolean secondCoralBannerTripped = false;

        public double coralVelocity = 0.0;
        public double coralAppliedVoltage = 0.0;
        public double coralSupplyCurrent = 0.0;
        public double coralStatorCurrent = 0.0;
        public double coralTemperature = 0.0;
    }

    public default void updateInputs(CoralIOInputs inputs) {}

    public default void runVolts(double voltage) {}

    public default void runVelocitySetpoint(double velocity) {}

    public default void stop() {}

    public default void setCurrentLimit(double currentLimit) {}

    public default void setBrakeMode(boolean enabled) {}

    @Override
    public default void refreshData() {}
}
