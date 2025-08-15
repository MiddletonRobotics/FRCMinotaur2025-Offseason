package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static frc.robot.utilities.PhoenixUtility.tryUntilOk;
import static edu.wpi.first.units.Units.Celsius;
import static frc.robot.utilities.PhoenixUtility.registerSignals;
import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.constants.DrivetrainConstants.ModuleConfiguration;

import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder swerveEncoder;

    private final Queue<Double> timestampQueue;

    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;
    private final StatusSignal<Voltage> driveAppliedVoltage;
    private final StatusSignal<Current> driveCurrent;
    private final StatusSignal<Temperature> driveTempurature;

    private final StatusSignal<Angle> steerPosition;
    private final Queue<Double> steerPositionQueue;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<AngularAcceleration> steerAcceleration;
    private final StatusSignal<Voltage> steerAppliedVoltage;
    private final StatusSignal<Current> steerCurrent;
    private final StatusSignal<Temperature> steerTempurature;

    private final StatusSignal<MagnetHealthValue> swerveEncoderMagnetHealth;
    private final StatusSignal<Angle> swerveEncoderPosition;

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0).withUpdateFreqHz(0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0).withUpdateFreqHz(0);
    private final VoltageOut voltageOutRequest = new VoltageOut(0).withUpdateFreqHz(0);
    private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

    private final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
    private final CANcoderConfiguration swerveEncoderConfiguration = new CANcoderConfiguration();

    private final Rotation2d absoluteEncoderOffset;

    public ModuleIOTalonFX(ModuleConfiguration configuration) {
        driveMotor = new TalonFX(configuration.driveMotorID(), "*");
        steerMotor = new TalonFX(configuration.steerMotorID(), "*");
        swerveEncoder = new CANcoder(configuration.swerveEncoderID(), "*");
        absoluteEncoderOffset = configuration.swerveEncoderOffset();

        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfiguration.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
        driveConfiguration.Feedback.SensorToMechanismRatio = DrivetrainConstants.kDriveGearRatio;
        driveConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = DrivetrainConstants.kDriveCurrentLimit.baseUnitMagnitude();
        driveConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -DrivetrainConstants.kDriveCurrentLimit.baseUnitMagnitude();
        driveConfiguration.CurrentLimits.StatorCurrentLimit = DrivetrainConstants.kDriveCurrentLimit.baseUnitMagnitude();
        driveConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfiguration));
        tryUntilOk(5, () -> driveMotor.setPosition(0.0));

        swerveEncoderConfiguration.MagnetSensor.MagnetOffset = configuration.swerveEncoderOffset().getRotations();
        swerveEncoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        swerveEncoderConfiguration.MagnetSensor.SensorDirection = configuration.swerveEncoderInverted() ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> swerveEncoder.getConfigurator().apply(swerveEncoderConfiguration));

        steerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfiguration.MotorOutput.Inverted = configuration.steerInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        steerConfiguration.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
        steerConfiguration.Feedback.FeedbackRemoteSensorID = configuration.swerveEncoderID();
        steerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfiguration.Feedback.SensorToMechanismRatio = DrivetrainConstants.kTurnGearRatio;
        steerConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        steerConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = DrivetrainConstants.kSteerCurrentLimit.baseUnitMagnitude();
        steerConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -DrivetrainConstants.kSteerCurrentLimit.baseUnitMagnitude();
        steerConfiguration.CurrentLimits.StatorCurrentLimit = DrivetrainConstants.kSteerCurrentLimit.baseUnitMagnitude();
        steerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfiguration));
        tryUntilOk(5, () -> steerMotor.setPosition(swerveEncoder.getAbsolutePosition().getValueAsDouble()));

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        drivePosition = driveMotor.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition().clone());
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();
        driveAppliedVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getSupplyCurrent();
        driveTempurature = driveMotor.getDeviceTemp();

        steerPosition = steerMotor.getPosition();
        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerMotor.getPosition().clone());
        steerVelocity = steerMotor.getVelocity();
        steerAcceleration = steerMotor.getAcceleration();
        steerAppliedVoltage = steerMotor.getMotorVoltage();
        steerCurrent = steerMotor.getSupplyCurrent();
        steerTempurature = steerMotor.getDeviceTemp();

        swerveEncoderMagnetHealth = swerveEncoder.getMagnetHealth();
        swerveEncoderPosition = swerveEncoder.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(DrivetrainConstants.kOdometryFrequency, drivePosition, steerPosition, swerveEncoderPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAcceleration,
            driveAppliedVoltage,
            driveCurrent,
            driveTempurature,
            steerVelocity,
            steerAcceleration,
            steerAppliedVoltage,
            steerCurrent,
            steerTempurature,
            swerveEncoderMagnetHealth
        );

        registerSignals(
            true, 
            drivePosition,
            driveVelocity,
            driveAcceleration,
            driveAppliedVoltage,
            driveCurrent,
            driveTempurature,
            steerPosition,
            steerVelocity,
            steerAcceleration,
            steerAppliedVoltage,
            steerCurrent,
            steerTempurature,
            swerveEncoderMagnetHealth,
            swerveEncoderPosition
        );
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveAcceleration,
            driveAppliedVoltage,
            driveCurrent,
            driveTempurature,
            steerPosition,
            steerVelocity,
            steerAcceleration,
            steerAppliedVoltage,
            steerCurrent,
            steerTempurature,
            swerveEncoderMagnetHealth,
            swerveEncoderPosition
        );

        inputs.isDriveMotorConnected = BaseStatusSignal.isAllGood(drivePosition, driveVelocity, driveAppliedVoltage, driveCurrent);
        inputs.drivePositionRadians = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocityRadiansPerSecond = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAccelerationRadiansPerSecondSquared = Units.rotationsToRadians(driveAcceleration.getValueAsDouble());
        inputs.driveAppliedVoltage = driveAppliedVoltage.getValueAsDouble();
        inputs.driveCurrentAmperes = new double[] {driveCurrent.getValueAsDouble()};
        inputs.driveTemperature = driveTempurature.getValueAsDouble();

        inputs.isSteerMotorConnected = BaseStatusSignal.isAllGood(steerPosition, steerVelocity, steerAppliedVoltage, steerCurrent);
        inputs.steerPosition = Rotation2d.fromRotations(steerPosition.getValueAsDouble());
        inputs.steerVelocityRadiansPerSecond = Units.rotationsToRadians(steerVelocity.getValueAsDouble());
        inputs.steerAccelerationRadiansPerSecondSquared = Units.rotationsToRadians(steerAcceleration.getValueAsDouble());
        inputs.steerAppliedVoltage = steerAppliedVoltage.getValueAsDouble();
        inputs.steerCurrentAmperes = new double[] {steerCurrent.getValueAsDouble()};
        inputs.steerTemperature = steerTempurature.getValueAsDouble();

        inputs.isSwerveEncoderConnected = BaseStatusSignal.isAllGood(swerveEncoderPosition, swerveEncoderMagnetHealth);
        inputs.swerveEncoderMagnetHealth = swerveEncoder.getMagnetHealth().getValue();
        inputs.swerveEncoderPosition = Rotation2d.fromRotations(swerveEncoderPosition.getValueAsDouble()).minus(absoluteEncoderOffset);

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometryDrivePositionsRadians = drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
        inputs.odometrySteerPositions = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveMotor.setControl(voltageOutRequest.withOutput(output * 12));
    }

    @Override
    public void setSteerOpenLoop(double output) {
        steerMotor.setControl(voltageOutRequest.withOutput(output * 12));
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveMotor.setControl(velocityVoltageRequest.withVelocity(Units.radiansToRotations(velocityRadPerSec)).withFeedForward(feedforward));
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerMotor.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));
    }

    @Override
    public void setDrivePIDCoefficients(double kP, double kI, double kD) {
        driveConfiguration.Slot0.kP = kP;
        driveConfiguration.Slot0.kI = kI;
        driveConfiguration.Slot0.kD = kD;
        tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfiguration));
    }

    @Override
    public void setSteerPIDCoefficients(double kP, double kI, double kD) {
        steerConfiguration.Slot0.kP = kP;
        steerConfiguration.Slot0.kI = kI;
        steerConfiguration.Slot0.kD = kD;
        tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfiguration));
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        brakeModeExecutor.execute(() -> {
            synchronized (driveConfiguration) {
                driveConfiguration.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfiguration));
            }
        });

        brakeModeExecutor.execute(() -> {
            synchronized (steerConfiguration) {
                steerConfiguration.MotorOutput.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
                tryUntilOk(5, () -> steerMotor.getConfigurator().apply(steerConfiguration));
            }
        });
    }

    @Override
    public boolean isDriveOverheating() {
        return driveTempurature.getValue().in(Celsius) > 90;
    }

    @Override
    public boolean isSteerOverheating() {
        return steerTempurature.getValue().in(Celsius) > 90;
    }
}
