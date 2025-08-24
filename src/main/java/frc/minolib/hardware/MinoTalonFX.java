package frc.minolib.hardware;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DriverStation;

import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.MinoStatusSignal;
import frc.minolib.phoenix.PIDConfiguration;
import frc.minolib.phoenix.PhoenixUtility;
import frc.minolib.io.MotorIO;
import frc.minolib.io.MotorInputsAutoLogged;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

public class MinoTalonFX implements  AutoCloseable, MotorIO {
    private static final double kCANTimeoutS = 0.1; // s
    private final String name;
    private final String loggingName;
    private final TalonFX controller;
    private final TalonFXSimState simulationState;
    private final MechanismRatio gearRatio;
    private final MinoTalonFXConfiguration configuration;

    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    private final VoltageOut voltageControl = new VoltageOut(0);
    private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private final MotionMagicVoltage motionMagicControl = new MotionMagicVoltage(0);
    private final DynamicMotionMagicVoltage dynamicMotionMagicControl = new DynamicMotionMagicVoltage(0, 0, 0, 0);

    private final MinoStatusSignal<Integer> faultFieldSignal;
    private final MinoStatusSignal<Integer> stickyFaultFieldSignal;
    private final MinoStatusSignal<Double> percentOutputSignal;
    private final MinoStatusSignal<Current> supplyCurrentSignal;
    private final MinoStatusSignal<Current> statorCurrentSignal;
    private final MinoStatusSignal<Current> torqueCurrentSignal;
    private final MinoStatusSignal<Angle> rotorPositionSignal;
    private final MinoStatusSignal<Angle> sensorPositionSignal;
    private final MinoStatusSignal<AngularVelocity> sensorVelocitySignal;
    private final MinoStatusSignal<Double> closedLoopReferenceSignal;
    private final MinoStatusSignal<Double> closedLoopReferenceSlopeSignal;
    private final MinoStatusSignal<Temperature> temperatureSignal;
    private final BaseStatusSignal[] allSignals;

    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    public static class MinoTalonFXConfiguration {
        private NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        private boolean INVERTED = false;
        private double SUPPLY_CURRENT_LIMIT = 40.0; // A
        private double STATOR_CURRENT_LIMIT = 40.0; // A
        private boolean FWD_SOFT_LIMIT_ENABLED = false;
        private double FWD_SOFT_LIMIT = 0.0; // In MechanismRatio units
        private boolean REV_SOFT_LIMIT_ENABLED = false;
        private double REV_SOFT_LIMIT = 0.0; // In MechanismRatio units
        private PIDConfiguration slot0Configuration = new PIDConfiguration();
        private PIDConfiguration slot1Configuration = new PIDConfiguration();
        private PIDConfiguration slot2Configuration = new PIDConfiguration();
        private double motionMagicCruiseVelocity = 0.0; // In MechanismRatio units
        private double motionMagicAcceleration = 0.0; // In MechanismRatio units
        private double motionMagicJerk = 0.0; // In MechanismRatio units
        private double bootPositionOffset = 0.0; // In MechanismRatio units
        private double rotorBootOffset = 0.0; // In rotor rotations [-1, 1]

        public MinoTalonFXConfiguration setBrakeMode() {
            NEUTRAL_MODE = NeutralModeValue.Brake;
            return this;
        }

        public MinoTalonFXConfiguration setInverted(final boolean inverted) {
            INVERTED = inverted;
            return this;
        }

        public MinoTalonFXConfiguration setStatorCurrentLimit(final double amperes) {
            STATOR_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoTalonFXConfiguration setSupplyCurrentLimit(final double amperes) {
            SUPPLY_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoTalonFXConfiguration setForwardSoftLimit(final double position) {
            FWD_SOFT_LIMIT_ENABLED = true;
            FWD_SOFT_LIMIT = position;
            return this;
        }

        public MinoTalonFXConfiguration setReverseSoftLimit(final double position) {
            REV_SOFT_LIMIT_ENABLED = true;
            REV_SOFT_LIMIT = position;
            return this;
        }

        public MinoTalonFXConfiguration setPIDConfig(final int slot, final PIDConfiguration configuration) {
            switch (slot) {
                case 0:
                    slot0Configuration = configuration;
                    break;
                case 1:
                    slot1Configuration = configuration;
                    break;
                case 2:
                    slot2Configuration = configuration;
                    break;
                default:
                    throw new RuntimeException("Invalid PID slot " + slot);
            }

            return this;
        }

        public MinoTalonFXConfiguration setMotionMagicConfig(final double cruiseVelocity, final double acceleration, final double jerk) {
            motionMagicCruiseVelocity = cruiseVelocity;
            motionMagicAcceleration = acceleration;
            motionMagicJerk = jerk;
            return this;
        }

        public MinoTalonFXConfiguration setBootPositionOffset(final double position) {
            bootPositionOffset = position;
            return this;
        }

        public MinoTalonFXConfiguration setRotorBootOffset(final double position) {
            rotorBootOffset = position;
            return this;
        }

        public TalonFXConfiguration toTalonFXConfiguration(final Function<Double, Double> toNativeSensorPosition, final Function<Double, Double> toNativeSensorVelocity) {
            final TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NEUTRAL_MODE;
            config.MotorOutput.Inverted = INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.DutyCycleNeutralDeadband = 0.0;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLowerLimit = SUPPLY_CURRENT_LIMIT;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1; // s

            config.TorqueCurrent.PeakForwardTorqueCurrent = STATOR_CURRENT_LIMIT;
            config.TorqueCurrent.PeakReverseTorqueCurrent = -STATOR_CURRENT_LIMIT;
            config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

            config.Feedback.FeedbackRotorOffset = rotorBootOffset;

            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = FWD_SOFT_LIMIT_ENABLED;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = toNativeSensorPosition.apply(FWD_SOFT_LIMIT);
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = REV_SOFT_LIMIT_ENABLED;
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = toNativeSensorPosition.apply(REV_SOFT_LIMIT);

            config.Voltage.SupplyVoltageTimeConstant = 0.0;
            config.Voltage.PeakForwardVoltage = 16.0;
            config.Voltage.PeakReverseVoltage = -16.0;

            config.Slot0 = slot0Configuration.fillCTRE(new Slot0Configs());
            config.Slot1 = slot1Configuration.fillCTRE(new Slot1Configs());
            config.Slot2 = slot2Configuration.fillCTRE(new Slot2Configs());

            config.MotionMagic.MotionMagicCruiseVelocity = toNativeSensorVelocity.apply(motionMagicCruiseVelocity);
            config.MotionMagic.MotionMagicAcceleration = toNativeSensorVelocity.apply(motionMagicAcceleration);
            config.MotionMagic.MotionMagicJerk = toNativeSensorVelocity.apply(motionMagicJerk);

            return config;
        }
    }

    public static MinoTalonFXConfiguration makeDefaultConfig() {
        return new MinoTalonFXConfiguration();
    }

    /** Follower constructor */
    public MinoTalonFX(final CANDeviceID canID, final MinoTalonFX leader, final MinoTalonFXConfiguration config) {
        this(canID, leader.getMechanismRatio(), config);
        controller.setControl(new StrictFollower(leader.getDeviceID()));
    }

    /** Constructor with full configuration */
    public MinoTalonFX(final CANDeviceID canID, final MechanismRatio ratio, final MinoTalonFXConfiguration config) {
        name = "TalonFX " + canID.toString();
        loggingName = "Inputs/" + name;
        controller = new TalonFX(canID.deviceNumber, canID.CANbusName);
        simulationState = controller.getSimState();
        gearRatio = ratio;
        configuration = config;

        faultFieldSignal = new MinoStatusSignal<>(controller.getFaultField());
        stickyFaultFieldSignal = new MinoStatusSignal<>(controller.getStickyFaultField());
        percentOutputSignal = new MinoStatusSignal<>(controller.getDutyCycle());
        supplyCurrentSignal = new MinoStatusSignal<>(controller.getSupplyCurrent());
        statorCurrentSignal = new MinoStatusSignal<>(controller.getStatorCurrent());
        torqueCurrentSignal = new MinoStatusSignal<>(controller.getTorqueCurrent());
        rotorPositionSignal = new MinoStatusSignal<>(controller.getRotorPosition());
        sensorPositionSignal = new MinoStatusSignal<>(controller.getRotorPosition(), this::fromNativeSensorPosition);
        sensorVelocitySignal = new MinoStatusSignal<>(controller.getRotorVelocity(), this::fromNativeSensorVelocity);
        closedLoopReferenceSignal = new MinoStatusSignal<>(controller.getClosedLoopReference(), this::fromNativeSensorPosition);
        closedLoopReferenceSlopeSignal = new MinoStatusSignal<>(controller.getClosedLoopReferenceSlope(), this::fromNativeSensorVelocity);
        temperatureSignal = new MinoStatusSignal<>(controller.getDeviceTemp());
        allSignals = MinoStatusSignal.toBaseStatusSignals(
            faultFieldSignal,
            stickyFaultFieldSignal,
            percentOutputSignal,
            supplyCurrentSignal,
            statorCurrentSignal,
            torqueCurrentSignal,
            rotorPositionSignal,
            sensorPositionSignal,
            sensorVelocitySignal,
            closedLoopReferenceSignal,
            closedLoopReferenceSlopeSignal,
            temperatureSignal
        );

        // Clear reset flag and sticky faults.
        controller.hasResetOccurred();
        controller.clearStickyFaults();

        Logger.recordOutput("Configuration/" + name, setConfiguration());
    }

    public boolean setConfiguration() {
        boolean allSuccess = true;

        // Set motor controller configuration.
        final TalonFXConfiguration config = configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity);
        allSuccess &= PhoenixUtility.retryUntilSuccess(() -> controller.getConfigurator().apply(config, kCANTimeoutS), () -> {
            TalonFXConfiguration readConfig = new TalonFXConfiguration();
            controller.getConfigurator().refresh(readConfig, kCANTimeoutS);
            return PhoenixUtility.TalonFXConfigsEqual(config, readConfig);
        }, name + ": applyConfiguration");

        // Set update frequencies.
        final double kFaultUpdateFrequency = 4.0; // Hz
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> faultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> faultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            name + ": faultFieldSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> stickyFaultFieldSignal.setUpdateFrequency(kFaultUpdateFrequency, kCANTimeoutS),
            () -> stickyFaultFieldSignal.getAppliedUpdateFrequency() == kFaultUpdateFrequency,
            name + ": stickyFaultFieldSignal.setUpdateFrequency()"
        );

        final double kUpdateFrequency = 100.0; // Hz
        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> percentOutputSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> percentOutputSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": percentOutputSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> supplyCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> supplyCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": supplyCurrentSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> statorCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> statorCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": statorCurrentSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> torqueCurrentSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> torqueCurrentSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": torqueCurrentSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> rotorPositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> rotorPositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": rotorPositionSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> sensorPositionSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> sensorPositionSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": sensorPositionSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> sensorVelocitySignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> sensorVelocitySignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": sensorVelocitySignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> closedLoopReferenceSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> closedLoopReferenceSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": closedLoopReferenceSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> closedLoopReferenceSlopeSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> closedLoopReferenceSlopeSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": closedLoopReferenceSlopeSignal.setUpdateFrequency()"
        );

        allSuccess &= PhoenixUtility.retryUntilSuccess(
            () -> temperatureSignal.setUpdateFrequency(kUpdateFrequency, kCANTimeoutS),
            () -> temperatureSignal.getAppliedUpdateFrequency() == kUpdateFrequency,
            name + ": temperatureSignal.setUpdateFrequency()"
        );

        // Disable all signals that have not been explicitly defined.
        allSuccess &= PhoenixUtility.retryUntilSuccess(() -> controller.optimizeBusUtilization(0.0, kCANTimeoutS), name + ": optimizeBusUtilization");

        // Block until we get valid signals.
        allSuccess &= BaseStatusSignal.waitForAll(kCANTimeoutS, allSignals).isOK();

        // Check if unlicensed.
        allSuccess &= !controller.getStickyFault_UnlicensedFeatureInUse().getValue();
        return allSuccess;
    }

    public boolean checkFaultsAndReconfigureIfNecessary() {
        // TODO: Log other faults.
        if (controller.hasResetOccurred()) {
            DriverStation.reportError(name + ": reset occured", false);
            setConfiguration();
            return true;
        }

        return false;
    }

    public void close() {
        controller.close();
    }

    public int getDeviceID() {
        return controller.getDeviceID();
    }


    @Override
    public void updateInputs() {
        inputs.isMotorConnected = BaseStatusSignal.isAllGood(allSignals);
        inputs.faultField = faultFieldSignal.getRawValue();
        inputs.stickyFaultField = stickyFaultFieldSignal.getRawValue();
        inputs.percentOutput = percentOutputSignal.getUnitConvertedValue();
        inputs.supplyCurrent = supplyCurrentSignal.getUnitConvertedValue();
        inputs.statorCurrent = statorCurrentSignal.getUnitConvertedValue();
        inputs.torqueCurrent = torqueCurrentSignal.getUnitConvertedValue();
        inputs.closedLoopReference = closedLoopReferenceSignal.getUnitConvertedValue();
        inputs.closedLoopReferenceSlope = closedLoopReferenceSlopeSignal.getUnitConvertedValue();
        inputs.rotorPosition = rotorPositionSignal.getUnitConvertedValue();
        inputs.sensorPosition = sensorPositionSignal.getUnitConvertedValue();
        inputs.latencyCompensatedSensorPosition = MinoStatusSignal.getLatencyCompensatedValue(sensorPositionSignal, sensorVelocitySignal);
        inputs.sensorVelocity = sensorVelocitySignal.getUnitConvertedValue();
        inputs.temperature = temperatureSignal.getUnitConvertedValue();

        Logger.processInputs(loggingName, inputs);
    }

    public void setBrakeMode(final boolean on) {
        configuration.NEUTRAL_MODE = on ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        controller.getConfigurator().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).MotorOutput);
    }

    public void setStatorCurrentLimit(final double amps) {
        configuration.STATOR_CURRENT_LIMIT = amps;

        // TODO: Consider a shorter non-blocking timeout
        controller.getConfigurator().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).CurrentLimits, kCANTimeoutS);
    }

    public void setPercentOutput(final double percent) {
        dutyCycleControl.Output = percent;
        controller.setControl(dutyCycleControl);
    }

    public void setVoltageOutput(final double voltage) {
        voltageControl.Output = voltage;
        controller.setControl(voltageControl);
    }

    public void setCurrentOutput(final double current, final double maxAbsDutyCycle) {
        currentControl.Output = current;
        currentControl.MaxAbsDutyCycle = maxAbsDutyCycle;
        controller.setControl(currentControl);
    }

    public void setPositionSetpoint(final int slot, final double setpoint) {
        setPositionSetpoint(slot, setpoint, 0.0);
    }

    public void setPositionSetpoint(final int slot, final double setpoint, final double feedforwardVolts) {
        positionControl.Slot = slot;
        positionControl.Position = toNativeSensorPosition(setpoint);
        positionControl.FeedForward = feedforwardVolts;
        controller.setControl(positionControl);
    }

    public void setMotionMagicPositionSetpoint(final int slot, final double setpoint) {
        setMotionMagicPositionSetpoint(slot, setpoint, 0.0);
    }

    public void setMotionMagicPositionSetpoint(final int slot, final double setpoint, final double feedforwardVolts) {
        motionMagicControl.Slot = slot;
        motionMagicControl.Position = toNativeSensorPosition(setpoint);
        motionMagicControl.FeedForward = feedforwardVolts;
        controller.setControl(motionMagicControl);
    }

    public void setDynamicMotionMagicPositionSetpoint(final int slot, final double setpoint, final double velocity, final double acceleration, final double jerk) {
        setDynamicMotionMagicPositionSetpoint(slot, setpoint, velocity, acceleration, jerk, 0.0);
    }

    public void setDynamicMotionMagicPositionSetpoint(final int slot, final double setpoint, final double velocity, final double acceleration, final double jerk, final double feedforwardVolts) {
        dynamicMotionMagicControl.Slot = slot;
        dynamicMotionMagicControl.Position = toNativeSensorPosition(setpoint);
        dynamicMotionMagicControl.FeedForward = feedforwardVolts;
        dynamicMotionMagicControl.Velocity = toNativeSensorVelocity(velocity);
        dynamicMotionMagicControl.Acceleration = toNativeSensorVelocity(acceleration);
        dynamicMotionMagicControl.Jerk = toNativeSensorVelocity(jerk);
        controller.setControl(dynamicMotionMagicControl);
    }

    public void setVelocitySetpoint(final int slot, final double setpointVelocity) {
        setVelocitySetpoint(slot, setpointVelocity, 0.0, 0.0);
    }

    public void setVelocitySetpoint(
        final int slot, final double setpointVelocity, final double feedforwardVolts) {
        setVelocitySetpoint(slot, setpointVelocity, 0.0, feedforwardVolts);
    }

    public void setVelocitySetpoint(final int slot, final double setpointVelocity, final double setpointAccel, final double feedforwardVolts) {
        velocityControl.Slot = slot;
        velocityControl.Velocity = toNativeSensorVelocity(setpointVelocity);
        velocityControl.Acceleration = toNativeSensorVelocity(setpointAccel);
        velocityControl.FeedForward = feedforwardVolts;
        controller.setControl(velocityControl);
    }

    public double getPercentOutput() {
        return inputs.percentOutput;
    }

    public double getPhysicalPercentOutput() {
        return (getInverted() ? -1.0 : 1.0) * getPercentOutput();
    }

    public double getSupplyCurrent() {
        return inputs.supplyCurrent;
    }

    public double getStatorCurrent() {
        return inputs.statorCurrent;
    }

    public double getTorqueCurrent() {
        return inputs.torqueCurrent;
    }

    public double getClosedLoopReference() {
        return inputs.closedLoopReference;
    }

    public double getClosedLoopReferenceSlope() {
        return inputs.closedLoopReferenceSlope;
    }

    public double getMotorTemperature() {
        return inputs.temperature;
    }

    public boolean getInverted() {
        // This assumes that the config has been properly applied.
        return configuration.INVERTED;
    }

    public void zeroSensorPosition() {
        setSensorPosition(0.0);
    }

    public void setSensorPosition(final double pos) {
        // TODO: Handle zero offset internally.
        controller.setPosition(toNativeSensorPosition(pos));
    }

    public double getSensorPosition() {
        return inputs.sensorPosition;
    }

    public double getLatencyCompensatedSensorPosition() {
        return inputs.latencyCompensatedSensorPosition;
    }

    public double getSensorVelocity() {
        return inputs.sensorVelocity;
    }

    public MechanismRatio getMechanismRatio() {
        return gearRatio;
    }

    public double toNativeSensorPosition(final double pos) {
        return toNativeSensorPosition(pos, gearRatio, configuration.bootPositionOffset);
    }

    public static double toNativeSensorPosition(final double pos, final MechanismRatio mr, final double bootPositionOffset) {
        // Native position is rotations. There is 1 rotation per revolution (lol).
        return mr.mechanismPositionToSensorRadians(pos - bootPositionOffset) / (2.0 * Math.PI);
    }

    public double fromNativeSensorPosition(final double pos) {
        return (pos / toNativeSensorPosition(1.0, gearRatio, 0.0)) + configuration.bootPositionOffset;
    }

    public double toNativeSensorVelocity(final double vel) {
        return toNativeSensorVelocity(vel, gearRatio);
    }

    public static double toNativeSensorVelocity(final double vel, final MechanismRatio mr) {
        // Native velocity is rotations per second.
        return toNativeSensorPosition(vel, mr, 0.0);
    }

    public double fromNativeSensorVelocity(final double vel) {
        return vel / toNativeSensorVelocity(1.0);
    }

    public void setSimulatedSensorPositionAndVelocity(final double pos, final double vel, final double dt, final MechanismRatio mr) {
        // Convert position into rotations.
        final double rotations = toNativeSensorPosition(pos, mr, 0.0);
        // Convert velocity into rotations per second.
        final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
        // Simulated hardware is never inverted, so flip signs accordingly.
        final double sign = getInverted() ? -1.0 : 1.0;
        simulationState.setRotorVelocity(sign * rotationsPerSecond);
        simulationState.setRawRotorPosition(sign * rotations);
    }

    public void setSimulatedSensorVelocity(final double vel, final double dt, final MechanismRatio mr) {
        // Convert velocity into rotations per second.
        final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
        // Simulated hardware is never inverted, so flip signs accordingly.
        final double sign = getInverted() ? -1.0 : 1.0;
        simulationState.setRotorVelocity(sign * rotationsPerSecond);
        simulationState.addRotorPosition(sign * rotationsPerSecond * dt);
    }
}