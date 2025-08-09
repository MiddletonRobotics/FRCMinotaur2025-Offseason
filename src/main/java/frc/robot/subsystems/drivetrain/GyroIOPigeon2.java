package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static frc.robot.utilities.PhoenixUtility.tryUntilOk;
import static frc.robot.utilities.PhoenixUtility.registerSignals;
import frc.robot.utilities.constants.DrivetrainConstants;

import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<AngularVelocity> pitchVelocity;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<AngularVelocity> rollVelocity;

    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(DrivetrainConstants.kPigeonID);

        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        pitch = pigeon.getPitch();
        pitchVelocity = pigeon.getAngularVelocityXWorld();
        roll = pigeon.getRoll();
        rollVelocity = pigeon.getAngularVelocityYWorld();

        yaw.setUpdateFrequency(DrivetrainConstants.kOdometryFrequency);
        BaseStatusSignal.setUpdateFrequencyForAll(50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);

        pigeon.optimizeBusUtilization();
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());

        registerSignals(true, yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);
        tryUntilOk(5, () -> pigeon.setYaw(0.0));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSecond = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        inputs.pitchPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.pitchVelocityRadPerSecond = Units.degreesToRadians(yawVelocity.getValueAsDouble());
        inputs.rollPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.rollVelocityRadPerSecond = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double v) -> v).toArray();
        inputs.odometryYawPositions = yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}