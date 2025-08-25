package frc.minolib.hardware;

import java.util.function.Function;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.minolib.hardware.MinoTalonFX.MinoTalonFXConfiguration;
import frc.minolib.io.MotorIO;
import frc.minolib.io.MotorInputsAutoLogged;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.MinoStatusSignal;
import frc.minolib.phoenix.PIDConfiguration;

public class MinoSparkMax implements AutoCloseable, MotorIO {
    private static final double kCANTimeoutS = 0.1; // s
    private final String name;
    private final String loggingName;
    private final SparkMax controller;
    private final SparkMaxSim simulationState;
    private final MechanismRatio gearRatio;
    private final MinoSparkMaxConfiguration configuration;
    private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

    public static class MinoSparkMaxConfiguration {
        private IdleMode IDLE_MODE = IdleMode.kCoast;
        private boolean INVERTED = false;
        private int PRIMARY_CURRENT_LIMIT = 40; // A
        private double SECONDARY_CURRENT_LIMIT = 40.0; // A
        private boolean FWD_SOFT_LIMIT_ENABLED = false;
        private double FWD_SOFT_LIMIT = 0.0; // In MechanismRatio units
        private boolean REV_SOFT_LIMIT_ENABLED = false;
        private double REV_SOFT_LIMIT = 0.0; // In MechanismRatio units
        private SmartMotionConfig smartMotionConfiguration = new SmartMotionConfig();
        private MAXMotionConfig maxMotionConfiguration = new MAXMotionConfig();
        private ClosedLoopConfig closedLoopConfiguration = new ClosedLoopConfig();

        public MinoSparkMaxConfiguration setBrakeMode() {
            IDLE_MODE = IdleMode.kBrake;
            return this;
        }

        public MinoSparkMaxConfiguration setInverted(final boolean inverted) {
            INVERTED = inverted;
            return this;
        }

        public MinoSparkMaxConfiguration setPrimaryCurrentLimit(final int amperes) {
            PRIMARY_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoSparkMaxConfiguration setSecondaryCurrentLimit(final double amperes) {
            SECONDARY_CURRENT_LIMIT = amperes;
            return this;
        }

        public MinoSparkMaxConfiguration setForwardSoftLimit(final double position) {
            FWD_SOFT_LIMIT_ENABLED = true;
            FWD_SOFT_LIMIT = position;
            return this;
        }

        public MinoSparkMaxConfiguration setReverseSoftLimit(final double position) {
            REV_SOFT_LIMIT_ENABLED = true;
            REV_SOFT_LIMIT = position;
            return this;
        }

        public MinoSparkMaxConfiguration setPIDConfig(final int type, final PIDConfiguration configuration) {
            switch (type) {
                case 0:
                    maxMotionConfiguration = configuration;
                    break;
                case 1:
                    maxMotionConfiguration = configuration;
                    break;
                case 2:
                    maxMotionConfiguration = configuration;
                    break;
                default:
                    throw new RuntimeException("Invalid PID type " + type);
            }

            return this;
        }

        public SparkMaxConfig toTalonFXConfiguration(final Function<Double, Double> toNativeSensorPosition, final Function<Double, Double> toNativeSensorVelocity) {
            final SparkMaxConfig configuration = new SparkMaxConfig();
            configuration.inverted(INVERTED);
            configuration.idleMode(IDLE_MODE);

            configuration.smartCurrentLimit(PRIMARY_CURRENT_LIMIT);
            configuration.secondaryCurrentLimit(SECONDARY_CURRENT_LIMIT);

            configuration.closedLoop.
            configuration.Feedback.FeedbackRotorOffset = rotorBootOffset;

            configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = FWD_SOFT_LIMIT_ENABLED;
            configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = toNativeSensorPosition.apply(FWD_SOFT_LIMIT);
            configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = REV_SOFT_LIMIT_ENABLED;
            configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = toNativeSensorPosition.apply(REV_SOFT_LIMIT);

            configuration.Voltage.SupplyVoltageTimeConstant = 0.0;
            configuration.Voltage.PeakForwardVoltage = 16.0;
            configuration.Voltage.PeakReverseVoltage = -16.0;

            configuration.closedLoop.apply(new ClosedLoopConfig().d(1));
            configuration.Slot1 = slot1Configuration.fillCTRE(new Slot1Configs());
            configuration.Slot2 = slot2Configuration.fillCTRE(new Slot2Configs());

            return configuration;
        }
    }

    public static MinoSparkMaxConfiguration makeDefaultConfig() {
        return new MinoSparkMaxConfiguration();
    }

    public MinoSparkMax(final CANDeviceID canID, final MinoSparkMax leader, final MinoSparkMaxConfiguration configuration) {
        this(canID, leader.getMechanismRatio(), configuration);
        controller.setControl(new StrictFollower(leader.getDeviceID()));
    }

    /** Constructor with full configuration */
    public MinoSparkMax(final CANDeviceID canID, final MotorType motorType, final MechanismRatio ratio, final MinoSparkMaxConfiguration configuration) {
        name = "SparkMax " + canID.toString();
        loggingName = "Inputs/" + name;
        controller = new SparkMax(canID.deviceNumber, motorType);
        simulationState = new SparkMaxSim(controller, DCMotor.getNEO(1));
        gearRatio = ratio;
        this.configuration = configuration;

        Logger.recordOutput("Configuration/" + name, setConfiguration());
    }

    public boolean checkFaultsAndReconfigureIfNecessary() {
        // TODO: Log other faults.
        if (controller.hasActiveFault()) {
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
        return controller.getDeviceId();
    }

    public void setBrakeMode(final boolean on) {
        configuration.IDLE_MODE = on ? IdleMode.kBrake : IdleMode.kCoast;
        controller.get().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).MotorOutput);
    }

    public void setPrimaryCurrentLimit(final int amps) {
        configuration.PRIMARY_CURRENT_LIMIT = amps;

        // TODO: Consider a shorter non-blocking timeout
        controller.get().apply(configuration.toTalonFXConfiguration(this::toNativeSensorPosition, this::toNativeSensorVelocity).CurrentLimits, kCANTimeoutS);
    }

    public void setPercentOutput(final double percent) {
        controller.set(percent);
    }

    public void setVoltageOutput(final double voltage) {
        controller.setVoltage(voltage);
    }

    // TODO: PAST HERE NOT DONE

    public void setCurrentOutput(final double current, final double maxAbsDutyCycle) {
        currentControl.Output = current;
        currentControl.MaxAbsDutyCycle = maxAbsDutyCycle;
        controller.set(currentControl);
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

    public void setSensorPosition(final double pos, final ControlType controlType) {
        controller.getClosedLoopController().setReference(toNativeSensorPosition(pos), controlType);
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
        simulationState.setVelocity(sign * rotationsPerSecond);
        simulationState.setPosition(sign * rotations);
    }

    public void setSimulatedSensorVelocity(final double vel, final double dt, final MechanismRatio mr) {
        // Convert velocity into rotations per second.
        final double rotationsPerSecond = toNativeSensorVelocity(vel, mr);
        // Simulated hardware is never inverted, so flip signs accordingly.
        final double sign = getInverted() ? -1.0 : 1.0;
        simulationState.setVelocity(sign * rotationsPerSecond);
        simulationState.setPosition(sign * rotationsPerSecond * dt);
    }
}
