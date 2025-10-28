package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ElevatorIO {
    public class ElevatorIOInputs {
        public double elevatorPositionInMeters = 0.0;

        public double elevatorAppliedVolts = 0.0;
        public double elevatorSupplyCurrentAmps = 0.0;
        public double elevatorStatorCurrentAmps = 0.0;
        public double elevatorVelocityMetersPerSec = 0.0;
        public double elevatorAccelerationMetersPerSecSquared = 0.0;

        public double elevatorRightMotorTemperture = 0.0;
        public double elevatorLeftMotorTemperture = 0.0;

        public boolean leftLimitSwitchTriggered = false;
        public boolean rightLimitSwitchTrigger = false;
    }

    public default void updateInputs(ElevatorIOInputs inputs) {};

    public default void setTargetPosition(double positionInMeters) {};

    public default void resetPosition(double positionInMeters) {};

    public default void setNeutralMode(NeutralModeValue neutralMode) {};

    public default void setDutyCycle(double dutyCycle) {};

    public default void refreshData() {};
}