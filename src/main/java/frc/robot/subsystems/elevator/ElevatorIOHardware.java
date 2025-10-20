package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOHardware implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private Follower followControlRequest;

    private DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0);
    private MotionMagicExpoVoltage positionVoltage = new MotionMagicExpoVoltage(0).withSlot(0);

    private final StatusSignal<Angle> elevatorPositionInMeters;
    private final StatusSignal<Voltage> elevatorAppliedVolts;
    private final StatusSignal<Current> elevatorSupplyCurrentAmps;
    private final StatusSignal<Current> elevatorStatorCurrentAmps;
    private final StatusSignal<AngularVelocity> elevatorVelocityMetersPerSec;
    private final StatusSignal<AngularAcceleration> elevatorAccelerationMetersPerSecSquared;
    private final StatusSignal<Temperature> rightMotorTempurture;
    private final StatusSignal<Temperature> leftMotorTempurture;

    public ElevatorIOHardware() {
        rightMotor = new TalonFX(ElevatorConstants.rightMotorID.deviceNumber, ElevatorConstants.rightMotorID.CANbusName);
        leftMotor = new TalonFX(ElevatorConstants.leftMotorID.deviceNumber, ElevatorConstants.leftMotorID.CANbusName);
        followControlRequest = new Follower(ElevatorConstants.rightMotorID.deviceNumber, true);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.StatorCurrentLimit = 120.0;

        config.Slot0.kP = 3.4;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        config.Slot0.kS = 0.9;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        config.Slot0.kG = 1.05;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.MotionMagic.MotionMagicAcceleration = 5.0 / (2 * Math.PI * Units.inchesToMeters(((16.0 * 0.25) / Math.PI))) * 6;
        config.MotionMagic.MotionMagicCruiseVelocity = 5.0 / (2 * Math.PI * Units.inchesToMeters(((16.0 * 0.25) / Math.PI))) * 6;

        rightMotor.getConfigurator().apply(config);
        leftMotor.getConfigurator().apply(config);

        leftMotor.setControl(followControlRequest);

        elevatorPositionInMeters = rightMotor.getPosition();
        elevatorAppliedVolts = rightMotor.getMotorVoltage();
        elevatorSupplyCurrentAmps = rightMotor.getSupplyCurrent();
        elevatorStatorCurrentAmps = rightMotor.getStatorCurrent();
        elevatorVelocityMetersPerSec = rightMotor.getRotorVelocity();
        elevatorAccelerationMetersPerSecSquared = rightMotor.getAcceleration();
        rightMotorTempurture = rightMotor.getDeviceTemp();
        leftMotorTempurture = leftMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPositionInMeters = Units.inchesToMeters(elevatorPositionInMeters.getValue().in(Rotations) * (1/2) * (Math.PI * (16 * 0.25 / Math.PI)));

        inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValueAsDouble();
        inputs.elevatorSupplyCurrentAmps = elevatorSupplyCurrentAmps.getValueAsDouble();
        inputs.elevatorStatorCurrentAmps = elevatorStatorCurrentAmps.getValueAsDouble();
        inputs.elevatorVelocityMetersPerSec = inputs.elevatorPositionInMeters / 60;
        inputs.elevatorAccelerationMetersPerSecSquared = inputs.elevatorPositionInMeters / 60 / 60;

        inputs.elevatorRightMotorTemperture = rightMotorTempurture.getValueAsDouble();
        inputs.elevatorLeftMotorTemperture = leftMotorTempurture.getValueAsDouble();
    }

    @Override
    public void setTargetPosition(double positionInMeters) {
        rightMotor.setControl(positionVoltage.withPosition(Units.metersToInches((2 * positionInMeters) / (Math.PI * (16 * 0.25 / Math.PI)))));
    }

    @Override
    public void resetPosition(double positionInMeters) {
        rightMotor.setPosition(Units.metersToInches((2 * positionInMeters) / (Math.PI * (16 * 0.25 / Math.PI))));
    }

    @Override
    public void setNeutralMode(NeutralModeValue neutralMode) {
        rightMotor.setNeutralMode(neutralMode);
        leftMotor.setNeutralMode(neutralMode);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        rightMotor.setControl(dutyCycleOut.withOutput(dutyCycle));
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            elevatorPositionInMeters,
            elevatorAppliedVolts,
            elevatorSupplyCurrentAmps,
            elevatorStatorCurrentAmps,
            elevatorVelocityMetersPerSec,
            elevatorAccelerationMetersPerSecSquared,
            rightMotorTempurture,
            leftMotorTempurture
        );
    }
}
