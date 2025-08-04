package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
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

import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.constants.GlobalConstants;

public class ModuleIOTalonFX implements ModuleIO {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;    
    private final CANcoder swerveEncoder;

    private final Queue<Double> timestampQueue;
    
    private final StatusSignal<ConnectedMotorValue> driveMotorConnected;
    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<AngularAcceleration> driveAcceleration;
    private final StatusSignal<Voltage> driveAppliedVoltage;
    private final StatusSignal<Current> driveCurrent;
    private final StatusSignal<Temperature> driveTempurature;

    private final StatusSignal<ConnectedMotorValue> steerMotorConnected;
    private final StatusSignal<MagnetHealthValue> steerEncoderMagnetHealth;
    private final StatusSignal<Angle> steerAbsolutePosition;
    private final StatusSignal<Angle> steerPosition;
    private final Queue<Double> steerPositionQueue;
    private final StatusSignal<AngularVelocity> steerVelocity;
    private final StatusSignal<Voltage> steerAppliedVoltage;
    private final StatusSignal<Current> steerCurrent;
    private final StatusSignal<Temperature> steerTempurature;
    private final StatusSignal<Voltage> swerveEncoderSupplyVoltage; // used for checking if the absolute is connected

    private final boolean isDriveMotorInverted = false;
    private final boolean isSteerMotorInverted = true;
    private final boolean isSwerveEncoderInverted = false;
    private final Rotation2d swerveEncoderOffset;

    private final VelocityVoltage driveControl = new VelocityVoltage(0.0).withEnableFOC(false);
    private final PositionVoltage steerControl = new PositionVoltage(0.0).withEnableFOC(false);
    private final VoltageOut rawControl = new VoltageOut(0.0).withEnableFOC(false);

    private final TalonFXConfiguration driveConfiguration;
    private final TalonFXConfiguration steerConfiguration;
    private final CANcoderConfiguration swerveEncoderConfiguration;

    public ModuleIOTalonFX(int index) {
        switch(index) {
            case 0:
                driveMotor = new TalonFX(DrivetrainConstants.Ports.kFrontLeftDriveMotorID);
                steerMotor = new TalonFX(DrivetrainConstants.Ports.kFrontLeftSteerMotorID);
                swerveEncoder = new CANcoder(DrivetrainConstants.Ports.kFrontLeftSwerveEncoderID);
                swerveEncoderOffset = new Rotation2d();
                break;
            case 1:
                driveMotor = new TalonFX(DrivetrainConstants.Ports.kFrontRightDriveMotorID);
                steerMotor = new TalonFX(DrivetrainConstants.Ports.kFrontRightSteerMotorID);
                swerveEncoder = new CANcoder(DrivetrainConstants.Ports.kFrontRightSwerveEncoderID);
                swerveEncoderOffset = new Rotation2d();
                break;
            case 2:
                driveMotor = new TalonFX(DrivetrainConstants.Ports.kBackLeftDriveMotorID);
                steerMotor = new TalonFX(DrivetrainConstants.Ports.kBackLeftSteerMotorID);
                swerveEncoder = new CANcoder(DrivetrainConstants.Ports.kBackLeftSwerveEncoderID);
                swerveEncoderOffset = new Rotation2d();
                break;
            case 3:
                driveMotor = new TalonFX(DrivetrainConstants.Ports.kBackRightDriveMotorID);
                steerMotor = new TalonFX(DrivetrainConstants.Ports.kBackRightSteerMotorID);
                swerveEncoder = new CANcoder(DrivetrainConstants.Ports.kBackRightSwerveEncoderID);
                swerveEncoderOffset = new Rotation2d();
                break;
            default:
                throw new RuntimeException("Invalid Index Number for: " + this.getClass().getName());
        }

        CurrentLimitsConfigs driveCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(DrivetrainConstants.kDriveDefaultSupplyCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(DrivetrainConstants.kDriveDefaultStatorCurrentLimit);

        CurrentLimitsConfigs steerCurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(DrivetrainConstants.kSteerDefaultSupplyCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(DrivetrainConstants.kSteerDefaultStatorCurrentLimit);

        Slot0Configs driveSlot0Configuration = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0);

        Slot0Configs steerSlot0Configuration = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0);

        FeedbackConfigs driveFeedbackConfiguration = new FeedbackConfigs()
            .withSensorToMechanismRatio(0);

        FeedbackConfigs steerFeedbackConfiguration = new FeedbackConfigs()
            .withSensorToMechanismRatio(0);

        TorqueCurrentConfigs driveTorque = new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(DrivetrainConstants.kDriveDefaultSupplyCurrentLimit.baseUnitMagnitude())
            .withPeakReverseTorqueCurrent(-DrivetrainConstants.kDriveDefaultSupplyCurrentLimit.baseUnitMagnitude());

        TorqueCurrentConfigs steerTorque = new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(DrivetrainConstants.kSteerDefaultSupplyCurrentLimit.baseUnitMagnitude())
            .withPeakReverseTorqueCurrent(-DrivetrainConstants.kSteerDefaultSupplyCurrentLimit.baseUnitMagnitude());

        ClosedLoopRampsConfigs driveClosedLoopRamp = new ClosedLoopRampsConfigs()
            .withTorqueClosedLoopRampPeriod(0.02);

        ClosedLoopGeneralConfigs steerClosedLoopGeneral = new ClosedLoopGeneralConfigs()
            .withContinuousWrap(true);

        driveConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(driveCurrentLimits)
            .withSlot0(driveSlot0Configuration)
            .withFeedback(driveFeedbackConfiguration)
            .withTorqueCurrent(driveTorque)
            .withClosedLoopRamps(driveClosedLoopRamp);

        driveMotor.getConfigurator().apply(driveConfiguration);
        driveMotor.setPosition(0.0);
        setDriveBrakeMode(true);

        steerConfiguration = new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(steerCurrentLimits)
            .withSlot0(steerSlot0Configuration)
            .withFeedback(steerFeedbackConfiguration)
            .withTorqueCurrent(steerTorque)
            .withClosedLoopGeneral(steerClosedLoopGeneral);

        steerMotor.getConfigurator().apply(steerConfiguration);
        setSteerBrakeMode(true);

        MagnetSensorConfigs currentMagnetConfiguration = new MagnetSensorConfigs();
        swerveEncoder.getConfigurator().refresh(currentMagnetConfiguration);

        currentMagnetConfiguration
            .withSensorDirection(isSwerveEncoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(Rotations.of(0.5));

        swerveEncoderConfiguration = new CANcoderConfiguration().withMagnetSensor(currentMagnetConfiguration);
        swerveEncoder.getConfigurator().apply(swerveEncoderConfiguration);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

        driveMotorConnected = driveMotor.getConnectedMotor();
        drivePosition = driveMotor.getPosition();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(driveMotor.getPosition());
        driveVelocity = driveMotor.getVelocity();
        driveAcceleration = driveMotor.getAcceleration();
        driveAppliedVoltage = driveMotor.getMotorVoltage();
        driveCurrent = driveMotor.getSupplyCurrent();
        driveTempurature = driveMotor.getDeviceTemp();

        steerMotorConnected = steerMotor.getConnectedMotor();
        steerEncoderMagnetHealth = swerveEncoder.getMagnetHealth();
        steerAbsolutePosition = swerveEncoder.getAbsolutePosition();
        steerPosition = steerMotor.getPosition();
        steerPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(steerMotor.getPosition());
        steerVelocity = steerMotor.getVelocity();
        steerAppliedVoltage = steerMotor.getMotorVoltage();
        steerCurrent = steerMotor.getSupplyCurrent();
        steerTempurature = steerMotor.getDeviceTemp();
        swerveEncoderSupplyVoltage = swerveEncoder.getSupplyVoltage();
        
        BaseStatusSignal.setUpdateFrequencyForAll(DrivetrainConstants.kOdometryFrequency, drivePosition, steerPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            Hertz.of(50),
            driveMotorConnected,
            steerMotorConnected,
            driveVelocity,
            steerVelocity,
            driveAcceleration,
            driveAppliedVoltage,
            steerAppliedVoltage,
            driveCurrent,
            steerCurrent,
            driveTempurature,
            steerTempurature,
            steerAbsolutePosition,
            swerveEncoderSupplyVoltage,
            steerEncoderMagnetHealth
        );
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if (!GlobalConstants.kUseBaseRefreshManager) {
            BaseStatusSignal.refreshAll(
                driveMotorConnected,
                drivePosition,
                driveVelocity,
                driveAcceleration,
                driveAppliedVoltage,
                driveCurrent,
                steerMotorConnected,
                steerEncoderMagnetHealth,
                steerAbsolutePosition,
                steerPosition,
                steerVelocity,
                steerAppliedVoltage,
                steerCurrent,
                swerveEncoderSupplyVoltage
            );
        }

        inputs.drivePositionRadians = drivePosition.getValue().in(Radians);
        inputs.driveVelocityRadiansPerSecond = driveVelocity.getValue().in(RadiansPerSecond);
        inputs.driveAccelerationRadiansPerSecondSquared = driveAcceleration.getValue().in(RadiansPerSecondPerSecond);
        inputs.driveAppliedVolts = driveAppliedVoltage.getValue().in(Volts);
        inputs.driveCurrentAmps = driveCurrent.getValue().in(Amps);
        inputs.driveTempuratureCelsius = driveTempurature.getValue().in(Celsius);
        inputs.isDriveMotorConnected = driveMotorConnected.getValue() != ConnectedMotorValue.Unknown;

        inputs.steerAbsolutePosition = new Rotation2d(steerAbsolutePosition.getValue()).minus(swerveEncoderOffset);
        inputs.steerPositionRadians = new Rotation2d(steerPosition.getValue());
        inputs.steerVelocityRadiansPerSecond = steerVelocity.getValue().in(RadiansPerSecond);
        inputs.steerAppliedVolts = steerAppliedVoltage.getValue().in(Volts);
        inputs.steerCurrentAmps = steerCurrent.getValue().in(Amps);
        inputs.steerTempuratureCelsius = steerTempurature.getValue().in(Celsius);
        inputs.isSteerMotorConnected = steerMotorConnected.getValue() != ConnectedMotorValue.Unknown;
        inputs.isSteerEncoderConnected = steerEncoderMagnetHealth.getValue() != MagnetHealthValue.Magnet_Invalid || swerveEncoderSupplyVoltage.getValue().in(Volts) > 0.0;

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometryDrivePositionRadians = drivePositionQueue.stream().mapToDouble((Double v) -> Units.rotationsToRadians(v)).toArray();
        inputs.odometrySteerPositions = steerPositionQueue.stream().map((Double v) -> Rotation2d.fromRotations(v)).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        drivePositionQueue.clear();
        steerPositionQueue.clear();
    }

    @Override
    public void setDriveVelocity(double velocityRadiansPerSecond, double feedforward) {
        driveMotor.setControl(driveControl.withVelocity(RadiansPerSecond.of(velocityRadiansPerSecond)).withFeedForward(feedforward));
    }

    @Override
    public void setSteerPosition(Rotation2d position) {
        steerMotor.setControl(steerControl.withPosition(position.getMeasure()));
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {
        MotorOutputConfigs configuration = new MotorOutputConfigs();
        configuration.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(configuration, 0.0);
    }

    @Override
    public void setSteerBrakeMode(boolean enable) {
        MotorOutputConfigs configuration = new MotorOutputConfigs();
        configuration.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steerMotor.getConfigurator().apply(configuration, 0.0);
    }

    @Override
    public void setCurrentLimit(double supplyLimit) {
        driveMotor.getConfigurator().apply(driveConfiguration.CurrentLimits.withSupplyCurrentLimit(supplyLimit), 0.0);
        driveMotor.getConfigurator().apply(driveConfiguration.TorqueCurrent.withPeakForwardTorqueCurrent(supplyLimit).withPeakReverseTorqueCurrent(-supplyLimit), 0.0);
    }

    @Override
    public void setDriveRawOutput(double output) {
        driveMotor.setControl(rawControl.withOutput(output));
    }

    @Override
    public void setSteerRawOutput(double output) {
        steerMotor.setControl(rawControl.withOutput(output));
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        driveMotor.getConfigurator().apply(driveConfiguration.Slot0.withKP(kP).withKI(kI).withKD(kD).withKS(0), 0.0);
    }

    @Override
    public void setSteerPID(double kP, double kI, double kD) {
        steerMotor.getConfigurator().apply(steerConfiguration.Slot0.withKP(kP).withKI(kI).withKD(kD), 0.0);
    }

    @Override
    public void resetSteerMotor(Angle position) {
        steerMotor.getConfigurator().setPosition(position);
    }
}
