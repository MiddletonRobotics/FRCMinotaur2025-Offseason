package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double drivePositionRadians = 0.0;
        public double driveVelocityRadiansPerSecond = 0.0;
        public double driveAccelerationRadiansPerSecondSquared = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempuratureCelsius = 0.0;
        public boolean isDriveMotorConnected = false;

        public Rotation2d steerAbsolutePosition = new Rotation2d();
        public Rotation2d steerPositionRadians = new Rotation2d();
        public double steerVelocityRadiansPerSecond = 0.0;
        public double steerAccelerationRadiansPerSecondSquared = 0.0;
        public double steerAppliedVolts = 0.0;
        public double steerCurrentAmps = 0.0;
        public double steerTempuratureCelsius = 0.0;
        public boolean isSteerMotorConnected = false;
        public boolean isSteerEncoderConnected = false;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionRadians = new double[] {};
        public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
    }

    /**
     * Updates the set of logged variables defined above
     * @param inputs
     */
    public void updateInputs(ModuleIOInputs inputs);

    /**
     * Set the setpoint velocity for the drive motor
     * @param velocityRadiansPerSecond The desired velocity for the drive motor
     * @param feedforward The feedforward model
     */
    public void setDriveVelocity(double velocityRadiansPerSecond, double feedforward);

    /**
     * Set the desired position for the steer motor
     * @param position Target position for the steer motor
     */
    public void setSteerPosition(Rotation2d position);

    /**
     * Switch between enabling and disabling brake mode on the drive motor
     * @param enable 
     */
    public void setDriveBrakeMode(boolean enable);

    /**
     * Switch between enabling and disabling brake mode on the steer motor
     * @param enable
     */
    public void setSteerBrakeMode(boolean enable);

    /**
     * Sets the current limits for both the drive motor and the steer motor
     * @param supplyLimits
     */
    public void setCurrentLimit(double supplyLimit);

    /**
     * Set the raw output for the drive motor
     * @param output
     */
    public void setDriveRawOutput(double output);

    /**
     * Set the raw output for the steer motor
     * @param output
     */
    public void setSteerRawOutput(double output);

    /**
     * Set the drive motor PID gains
     * @param kP Set the proportional gain
     * @param kI Set the integral gain
     * @param kD Set the derivative gain
     * @param kF Set the force-feedback gain
     */
    public void setDrivePID(double kP, double kI, double kD);

    /**
     * Set the steer motor PID gains
     * @param kP Set the proportional gain
     * @param kI Set the integral gain
     * @param kD Set the derivative gain
     * @param kF Set the force-feedback gain
     */
    public void setSteerPID(double kP, double kI, double kD);

    /**
     * Resets the steer motor position, which allows it to be resynced with the CANCoder
     * @param position The position to reset the steer motor to
     */
    public void resetSteerMotor(Angle position);
}
