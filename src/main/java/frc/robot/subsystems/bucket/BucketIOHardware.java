package frc.robot.subsystems.bucket;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

import static frc.minolib.rev.REVUtility.tryUntilOk;

import java.util.function.DoubleSupplier;

import static frc.minolib.rev.REVUtility.ifOkOrDefault;
import static edu.wpi.first.units.Units.Meters;
import static frc.minolib.phoenix.PhoenixUtility.retryUntilSuccess;

import frc.minolib.phoenix.PhoenixUtility;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.bucket.BucketIO.BucketIOInputs;

public class BucketIOHardware implements BucketIO {
    private final SparkBase rollerMotor;
    private final RelativeEncoder rollerEncoder;
    private final SparkBaseConfig rollerConfiguration;

    private final CANrange firstCANRange;
    private final CANrangeConfiguration canrangeConfiguration;
    private final DigitalInput secondBeamBreak;

    private final StatusSignal<Boolean> isDetected;
    private final StatusSignal<Distance> distance;
    private final StatusSignal<Double> signalStrength;

    private int currentLimit = 60;
    private boolean brakeModeEnabled = true;

    public BucketIOHardware(int deviceID, boolean isFlex) {
        rollerMotor = isFlex
            ? new SparkFlex(deviceID, SparkLowLevel.MotorType.kBrushless) 
            : new SparkMax(deviceID, SparkLowLevel.MotorType.kBrushless);

        rollerEncoder = rollerMotor.getEncoder();
        
        rollerConfiguration = isFlex ? new SparkFlexConfig() : new SparkMaxConfig();
        rollerConfiguration
            .idleMode(brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
            .smartCurrentLimit(currentLimit, 50)
            .voltageCompensation(12.0);

        rollerConfiguration.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
        rollerConfiguration.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);

        tryUntilOk(rollerMotor, 5, () -> rollerMotor.configure(rollerConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        tryUntilOk(rollerMotor, 5, () -> rollerEncoder.setPosition(0.0));

        firstCANRange = new CANrange(24);

        canrangeConfiguration = new CANrangeConfiguration()
            .withProximityParams(new ProximityParamsConfigs()
                .withProximityThreshold(0.11)
                .withProximityHysteresis(0.01)
                .withMinSignalStrengthForValidMeasurement(5500)
            ).withFovParams(new FovParamsConfigs());

        retryUntilSuccess(
            () -> firstCANRange.getConfigurator().apply(canrangeConfiguration), 
            () -> {
                CANrangeConfiguration readConfig = new CANrangeConfiguration();
                firstCANRange.getConfigurator().refresh(readConfig, 0.1);
                return PhoenixUtility.CANRangeConfigsEqual(canrangeConfiguration, readConfig);
            }, 
            5,
            "CANRange " + firstCANRange.getDeviceID() + ": applyConfiguration"
        );

        isDetected = firstCANRange.getIsDetected();
        distance = firstCANRange.getDistance();
        signalStrength = firstCANRange.getSignalStrength();

        secondBeamBreak = new DigitalInput(2);
    }

    @Override
    public void updateInputs(BucketIOInputs inputs) {
        inputs.bucketVelocity = ifOkOrDefault(rollerMotor, rollerEncoder::getVelocity, inputs.bucketVelocity);
        inputs.bucketAppliedVoltage = ifOkOrDefault(
            rollerMotor, 
            new DoubleSupplier[] {rollerMotor::getBusVoltage, rollerMotor::getAppliedOutput}, x -> x[0] * x[1],
            inputs.bucketAppliedVoltage
        );

        inputs.bucketSupplyCurrent = ifOkOrDefault(rollerMotor, rollerMotor::getOutputCurrent, inputs.bucketSupplyCurrent);
        inputs.bucketStatorCurrent = -1.0;
        inputs.bucketTemperature = ifOkOrDefault(rollerMotor, rollerMotor::getMotorTemperature, inputs.bucketTemperature);

        inputs.firstCANRangeTripped = isDetected.getValue().booleanValue();
        inputs.firstCANRangeSignalStrength = signalStrength.getValue().doubleValue();
        inputs.firstCANRangeDistanceInMeters = distance.getValue().in(Meters);
        inputs.secondCoralBannerTripped = !secondBeamBreak.get();
    }

    @Override
    public void runVolts(double voltage) {
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void runVelocitySetpoint(double velocity) {

    }

    @Override
    public void stop() {
        rollerMotor.stopMotor();
    }

    @Override
    public void setCurrentLimit(double currentLimit) {
        this.currentLimit = (int) currentLimit;
        
        new Thread(() ->
            tryUntilOk(rollerMotor, 5, () ->
                rollerMotor.configure(
                    rollerConfiguration.smartCurrentLimit(this.currentLimit, 50),
                    SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kNoPersistParameters
                )
            )
        ).start();
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        
        new Thread(() ->
            tryUntilOk(rollerMotor, 5, () ->
                rollerMotor.configure(
                    rollerConfiguration.idleMode(brakeModeEnabled ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast),
                    SparkBase.ResetMode.kNoResetSafeParameters,
                    SparkBase.PersistMode.kNoPersistParameters
                )
            )
        ).start();
    }

    @Override
    public void refreshData() {
        BaseStatusSignal.refreshAll(
            isDetected,
            distance,
            signalStrength
        );
    }
}
