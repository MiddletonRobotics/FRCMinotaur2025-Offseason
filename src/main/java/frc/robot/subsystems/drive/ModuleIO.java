package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.MagnetHealthValue;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean isDriveMotorConnected = false;
        public double drivePositionRadians = 0.0;
        public double driveTemperature = 0.0;
        public double driveVelocityRadiansPerSecond = 0.0;
        public double driveAccelerationRadiansPerSecondSquared = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double[] driveCurrentAmperes = new double[] {};

        public boolean isSteerMotorConnected = false;
        public double steerTemperature = 0.0;
        public Rotation2d steerPosition = new Rotation2d();
        public double steerVelocityRadiansPerSecond = 0.0;
        public double steerAccelerationRadiansPerSecondSquared = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double[] steerCurrentAmperes = new double[] {};

        public boolean isSwerveEncoderConnected = false;
        public MagnetHealthValue swerveEncoderMagnetHealth = MagnetHealthValue.Magnet_Invalid;
        public Rotation2d swerveEncoderPosition = new Rotation2d();

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRadians = new double[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputs inputs) {}

    /** Run the drive motor at the specified open loop value. */
    public default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    public default void setSteerOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    public default void setDriveVelocity(double velocityRadPerSec, double feedforward) {}

    /** Run the turn motor to the specified rotation. */
    public default void setSteerPosition(Rotation2d rotation) {}

    /** Set P, I, and D gains for closed loop control on drive motor. */
    public default void setDrivePIDCoefficients(double kP, double kI, double kD) {}

    /** Set P, I, and D gains for closed loop control on turn motor. */
    public default void setSteerPIDCoefficients(double kP, double kI, double kD) {}

    /** Set brake mode on drive motor */
    public default void setBrakeMode(boolean enabled) {}
}
