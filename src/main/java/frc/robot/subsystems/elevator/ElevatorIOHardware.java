package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

import static frc.minolib.phoenix.PhoenixUtility.retryUntilSuccess;
import frc.robot.constants.ElevatorConstants;

public class ElevatorIOHardware implements ElevatorIO {
    private TalonFX leftMotor;
    private TalonFX rightMotor;
    private Follower followControlRequest;

    private TalonFXConfiguration configuration;

    private VoltageOut dutyCycleOut = new VoltageOut(0.0);
    private MotionMagicExpoVoltage positionVoltage = new MotionMagicExpoVoltage(0).withSlot(0);

    private final StatusSignal<Angle> elevatorPositionRotations;
    private final StatusSignal<AngularVelocity> elevatorVelocityRotationsPerSecond;
    private final StatusSignal<AngularAcceleration> elevatorAccelerationRotationsPerSecondSquared;
    private final StatusSignal<Voltage> elevatorAppliedVoltage;
    private final StatusSignal<Current> elevatorSupplyCurrentAmperes;
    private final StatusSignal<Current> elevatorStatorCurrentAmperes;
    private final StatusSignal<Temperature> rightMotorTempurture;
    private final StatusSignal<Temperature> leftMotorTempurture;

    public ElevatorIOHardware() {
        rightMotor = new TalonFX(ElevatorConstants.rightMotorID.deviceNumber, ElevatorConstants.rightMotorID.CANbusName);
        leftMotor = new TalonFX(ElevatorConstants.leftMotorID.deviceNumber, ElevatorConstants.leftMotorID.CANbusName);
        followControlRequest = new Follower(ElevatorConstants.rightMotorID.deviceNumber, true);

        configuration = new TalonFXConfiguration();

        configuration.CurrentLimits.SupplyCurrentLimitEnable = true;
        configuration.CurrentLimits.SupplyCurrentLimit = 60.0;
        configuration.CurrentLimits.StatorCurrentLimitEnable = true;
        configuration.CurrentLimits.StatorCurrentLimit = 120.0;

        configuration.Slot0.kP = 3.4;
        configuration.Slot0.kI = 0.0;
        configuration.Slot0.kD = 0.0;

        configuration.Slot0.kS = 0.9;
        configuration.Slot0.kV = 0.0;
        configuration.Slot0.kA = 0.0;
        configuration.Slot0.kG = 1.05;
        configuration.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        configuration.MotionMagic.MotionMagicAcceleration = 5.0 / (2 * Math.PI * Units.inchesToMeters(((16.0 * 0.25) / Math.PI))) * 6;
        configuration.MotionMagic.MotionMagicCruiseVelocity = 5.0 / (2 * Math.PI * Units.inchesToMeters(((16.0 * 0.25) / Math.PI))) * 6;

        rightMotor.getConfigurator().apply(configuration);
        leftMotor.getConfigurator().apply(configuration);

        leftMotor.setControl(followControlRequest.withOpposeMasterDirection(true));

        elevatorPositionRotations = rightMotor.getPosition();
        elevatorVelocityRotationsPerSecond = rightMotor.getRotorVelocity();
        elevatorAccelerationRotationsPerSecondSquared = rightMotor.getAcceleration();
        elevatorAppliedVoltage = rightMotor.getMotorVoltage();
        elevatorSupplyCurrentAmperes = rightMotor.getSupplyCurrent();
        elevatorStatorCurrentAmperes = rightMotor.getStatorCurrent();
        rightMotorTempurture = rightMotor.getDeviceTemp();
        leftMotorTempurture = leftMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.elevatorPositionInMeters = elevatorRotationsToMeters(elevatorPositionRotations.getValue().in(Rotations));
        inputs.elevatorVelocityMetersPerSecond = elevatorRotationsToMeters(elevatorVelocityRotationsPerSecond.getValue().in(RotationsPerSecond));
        inputs.elevatorAccelerationMetersPerSecondSquared = elevatorRotationsToMeters(elevatorAccelerationRotationsPerSecondSquared.getValue().in(RotationsPerSecondPerSecond));

        inputs.elevatorAppliedVoltage = elevatorAppliedVoltage.getValue().in(Volts);
        inputs.elevatorSupplyCurrentAmperes = elevatorSupplyCurrentAmperes.getValue().in(Amps);
        inputs.elevatorStatorCurrentAmperes = elevatorStatorCurrentAmperes.getValue().in(Amps);

        inputs.elevatorRightMotorTemperture = rightMotorTempurture.getValue().in(Celsius);
        inputs.elevatorLeftMotorTemperture = leftMotorTempurture.getValue().in(Celsius);
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
    public void setPID(double kP, double kI, double kD) {
        configuration.Slot0.kP = kP;
        configuration.Slot0.kI = kI;
        configuration.Slot0.kD = kD;

        rightMotor.getConfigurator().apply(configuration);
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            elevatorPositionRotations,
            elevatorVelocityRotationsPerSecond,
            elevatorAccelerationRotationsPerSecondSquared,
            elevatorAppliedVoltage,
            elevatorSupplyCurrentAmperes,
            elevatorStatorCurrentAmperes,
            rightMotorTempurture,
            leftMotorTempurture
        );
    }

    public double elevatorRotationsToMeters(double motorRotations) {
        double sprocketRotations = motorRotations / 6.0;
        double circumferenceInches = Math.PI * (16 * 0.25);
        double elevatorTravelInches = sprocketRotations * circumferenceInches;
        double fullExtensionInches = elevatorTravelInches * 3.0;
    
        return Units.inchesToMeters(fullExtensionInches);
    }

    public double elevatorMetersToRotations(double heightMeters) {
        double heightInches = Units.metersToInches(heightMeters);
        double circumferenceInches = Math.PI * (16 * 0.25);
        double stageTravelInches = heightInches / 3.0;
        double sprocketRotations = stageTravelInches / circumferenceInches;
    
        return sprocketRotations * 6.0;
    }
}
