package frc.robot.subsystems.bucket;

import org.littletonrobotics.junction.AutoLog;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface BucketIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class BucketIOInputs {
        public boolean firstCANRangeTripped = false;
        public double firstCANRangeSignalStrength = 0.0;
        public double firstCANRangeDistanceInMeters = 0.0;
        public boolean secondCoralBannerTripped = false;

        public double bucketVelocity = 0.0;
        public double bucketAppliedVoltage = 0.0;
        public double bucketSupplyCurrent = 0.0;
        public double bucketStatorCurrent = 0.0;
        public double bucketTemperature = 0.0;
    }

    public default void updateInputs(BucketIOInputs inputs) {}

    public default void runVolts(double voltage) {}

    public default void runVelocitySetpoint(double velocity) {}

    public default void stop() {}

    public default void setCurrentLimit(double currentLimit) {}

    public default void setBrakeMode(boolean enabled) {}

    @Override
    public default void refreshData() {}
}
