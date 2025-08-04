package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public Rotation2d pitchPosition = new Rotation2d();

        public double[] odometryTimestamps = new double[] {};
        public Rotation2d[] odometryYawPosition = new Rotation2d[] {};
        public Rotation2d[] odometryPitchPosition = new Rotation2d[] {};

        public double yawVelocityRadiansPerSecond = 0.0;
        public double pitchVelocityRadiansPerSecond = 0.0;

        public double xAcceleration = 0.0;
        public double yAcceleration = 0.0;
        public double zAcceleration = 0.0;
    }

    public void updateInputs(GyroIOInputs inputs);
}
