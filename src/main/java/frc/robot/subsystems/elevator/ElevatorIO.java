package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean leaderMotorConnected = false;
        public double leaderPositionRadians = 0.0;
        public double leaderTemperature = 0.0;
        public double leaderVelocityRadiansPerSecond = 0.0;
        public double leaderAccelerationRadiansPerSecondSquared = 0.0;
        public double leaderAppliedVoltage = 0.0;
        public double[] leaderCurrentAmperes = new double[] {};

        public boolean followerMotorConnected = false;
        public double followerTemperature = 0.0;
        public double followerPositionRadians = 0.0;
        public double followerVelocityRadiansPerSecond = 0.0;
        public double followerAccelerationRadiansPerSecondSquared = 0.0;
        public double followerAppliedVoltage = 0.0;
        public double[] followerCurrentAmperes = new double[] {};
    }
}