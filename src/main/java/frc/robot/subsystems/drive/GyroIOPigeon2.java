package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.constants.GlobalConstants;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 gyro;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> pitchPositionQueue;
    private final Queue<Double> timestampQueue;

    private final StatusSignal<Angle> yawPosition;
    private final StatusSignal<Angle> pitchPosition;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final StatusSignal<AngularVelocity> pitchVelocity;
    private final StatusSignal<LinearAcceleration> xAcceleration;
    private final StatusSignal<LinearAcceleration> yAcceleration;
    private final StatusSignal<LinearAcceleration> zAcceleration;
    private final StatusSignal<Voltage> gyroSupplyVoltage;

    public GyroIOPigeon2() {
        gyro = new Pigeon2(DrivetrainConstants.Ports.kSwerveGyroID, DrivetrainConstants.Ports.kDrivetrainCanivoreName);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro.getYaw());
        pitchPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro.getPitch());

        yawPosition = gyro.getYaw();
        pitchPosition = gyro.getPitch();
        yawVelocity = gyro.getAngularVelocityZWorld();
        pitchVelocity = gyro.getAngularVelocityXWorld();
        xAcceleration = gyro.getAccelerationX();
        yAcceleration = gyro.getAccelerationY();
        zAcceleration = gyro.getAccelerationZ();
        gyroSupplyVoltage = gyro.getSupplyVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
            DrivetrainConstants.kOdometryFrequency,
            yawPosition,
            pitchPosition,
            yawVelocity,
            pitchVelocity,
            xAcceleration,
            yAcceleration,
            zAcceleration   
        );

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, gyroSupplyVoltage);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        if(!GlobalConstants.kUseBaseRefreshManager) {
            BaseStatusSignal.refreshAll(
                yawPosition,
                pitchPosition,
                yawVelocity,
                pitchVelocity,
                xAcceleration,
                yAcceleration,
                zAcceleration,
                gyroSupplyVoltage
            );
        }

        inputs.connected = gyroSupplyVoltage.getValue().in(Volts) != 0.0;

        if(RobotBase.isSimulation()) {
            inputs.connected = false;
        }

        inputs.yawPosition  = new Rotation2d(yawPosition.getValue());
        inputs.pitchPosition = new Rotation2d(pitchPosition.getValue());
        inputs.yawVelocityRadiansPerSecond = yawVelocity.getValue().in(RadiansPerSecond);
        inputs.pitchVelocityRadiansPerSecond = pitchVelocity.getValue().in(RadiansPerSecond);
        inputs.xAcceleration = xAcceleration.getValue().in(MetersPerSecondPerSecond);
        inputs.yAcceleration = yAcceleration.getValue().in(MetersPerSecondPerSecond);
        inputs.zAcceleration = zAcceleration.getValue().in(MetersPerSecondPerSecond);

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometryYawPosition = yawPositionQueue.stream().map((Double v) -> Rotation2d.fromDegrees(v)).toArray(Rotation2d[]::new);
        inputs.odometryPitchPosition = pitchPositionQueue.stream().map((Double v) -> Rotation2d.fromDegrees(v)).toArray(Rotation2d[]::new);

        timestampQueue.clear();
        yawPositionQueue.clear();
        pitchPositionQueue.clear();
    }
}
