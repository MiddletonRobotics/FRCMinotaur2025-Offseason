package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.minolib.utilities.SubsystemDataProcessor;

public interface ElevatorIO extends SubsystemDataProcessor.IODataRefresher {

    @AutoLog
    public class ElevatorIOInputs {
        public double elevatorPositionInMeters = 0.0;
        public double elevatorVelocityMetersPerSecond = 0.0;
        public double elevatorAccelerationMetersPerSecondSquared = 0.0;

        public double elevatorAppliedVoltage = 0.0;
        public double elevatorSupplyCurrentAmperes = 0.0;
        public double elevatorStatorCurrentAmperes = 0.0;

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

    public default void setPID(double kP, double kI, double kD) {};

    @Override
    public void refreshData();
}