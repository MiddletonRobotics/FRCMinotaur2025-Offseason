package frc.minolib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.minolib.hardware.CANDeviceID;
import frc.minolib.hardware.MinoCANCoder;
import frc.minolib.math.MathUtility;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.PIDConfiguration;
import frc.minolib.hardware.MinoTalonFX;
import frc.robot.Robot;

public class MinoSwerveModule {
    private static final int kDriveOpenLoopVelocityPIDSlot = 0;
    private static final int kDriveClosedLoopVelocityPIDSlot = 1;
    private static final int kSteeringPIDSlot = 0;

    private final Translation2d position;
    private final double absoluteEncoderOffset;
    protected final MinoTalonFX driveMotor;
    protected final MinoTalonFX steeringMotor;
    private final MechanismRatio driveRatio;
    private final MechanismRatio steeringRatio;
    // For every steering rotation with the wheel fixed, the drive motor turns this much. May be
    // negative.
    private final double steerDriveCouplingRatio;
    private final MinoCANCoder absoluteSteeringEncoder;
    private final double wheelCircumference;

    private SwerveModuleState m_lastCommandedState;
    private double m_steeringZeroPosition = 0.0;

    private double m_prevStateTime;

    public MinoSwerveModule(
        final Translation2d position,
        final CANDeviceID driveMotorID,
        final CANDeviceID steeringMotorID,
        final CANDeviceID absEncoderID,
        final PIDConfiguration driveOpenLoopPIDConfig,
        final PIDConfiguration driveClosedLoopPIDConfig,
        final PIDConfiguration steeringPIDConfig,
        final MechanismRatio driveRatio,
        final MechanismRatio steeringRatio,
        final double steerDriveCouplingRatio,
        final double absEncoderOffsetRadians,
        final double wheelCircumference
    ) {
        this.position = position;
        this.absoluteEncoderOffset = absEncoderOffsetRadians;
        driveMotor = new MinoTalonFX(driveMotorID, driveRatio, MinoTalonFX.makeDefaultConfig()
            .setBrakeMode()
            .setSupplyCurrentLimit(40)
            .setStatorCurrentLimit(120)
            .setPIDConfig(kDriveOpenLoopVelocityPIDSlot, driveOpenLoopPIDConfig)
            .setPIDConfig(kDriveClosedLoopVelocityPIDSlot, driveClosedLoopPIDConfig)
        );

        steeringMotor = new MinoTalonFX(steeringMotorID, steeringRatio, MinoTalonFX.makeDefaultConfig()
            .setInverted(true)
            .setSupplyCurrentLimit(30)
            .setStatorCurrentLimit(60)
            .setPIDConfig(kSteeringPIDSlot, steeringPIDConfig)
        );

        this.driveRatio = driveRatio;
        this.steeringRatio = steeringRatio;
        this.steerDriveCouplingRatio = steerDriveCouplingRatio;
        absoluteSteeringEncoder = new MinoCANCoder(absEncoderID, new MechanismRatio());
        this.wheelCircumference = wheelCircumference;

        zeroToAbsPosition();
        m_lastCommandedState = getState();

        // For simulation
        if (Robot.isSimulation()) {
            m_steeringSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1),
                steeringRatio.reduction(),
                0.03, // MOI
                0.0, // Length (m)
                Double.NEGATIVE_INFINITY, // Min angle
                Double.POSITIVE_INFINITY, // Max angle
                false, // Simulate gravity
                0.0 // Starting angle (rads)
            );
            
            m_simTimer = new Timer();
            m_simTimer.start();
        }
    }

    public void updateInputs() {
        absoluteSteeringEncoder.updateInputs();
        driveMotor.updateInputs();
        steeringMotor.updateInputs();
    }

    public Translation2d getPosition() {
        return position;
    }

    public void zeroToAbsPosition() {
        final double absAngle = absoluteSteeringEncoder.getAbsPosition() - absoluteEncoderOffset;
        m_steeringZeroPosition = steeringMotor.getSensorPosition() - absAngle;
    }

    public double getAbsoluteSensorAngle() {
        return absoluteSteeringEncoder.getAbsPosition();
    }

    public double getSteeringAngle() {
        return steeringMotor.getSensorPosition() - m_steeringZeroPosition;
    }

    public void setDesiredState(final SwerveModuleState desiredState, final boolean isClosedLoop) {
        double curTime = Timer.getTimestamp();
        double dT = curTime - m_prevStateTime;
        double a = m_prevStateTime != 0.0 ? ((desiredState.speedMetersPerSecond - m_lastCommandedState.speedMetersPerSecond) / dT) : 0.0;
        driveMotor.setVelocitySetpoint(
            isClosedLoop ? kDriveClosedLoopVelocityPIDSlot : kDriveOpenLoopVelocityPIDSlot,
            desiredState.speedMetersPerSecond,
            a,
            0.0
        );

        steeringMotor.setPositionSetpoint(kSteeringPIDSlot, desiredState.angle.getRadians() + m_steeringZeroPosition);

        // Save this state
        m_lastCommandedState = desiredState;
        m_prevStateTime = curTime;
    }

    public SwerveModuleState getState() {
        final double velocity = driveMotor.getSensorVelocity();
        final double angle = getSteeringAngle();
        return new SwerveModuleState(velocity, new Rotation2d(angle));
    }

    public SwerveModuleState getLastCommandedState() {
        return m_lastCommandedState;
    }

    public SwerveModulePosition getPositionState() {
        final double position = driveMotor.getLatencyCompensatedSensorPosition();
        final double angle =
            steeringMotor.getLatencyCompensatedSensorPosition() - m_steeringZeroPosition;

        // Compute drive position offset (in meters) due to steer coupling.
        final double steerCouplingDriveOffset = driveMotor.getMechanismRatio().sensorRadiansToMechanismPosition(steerDriveCouplingRatio * angle);

        return new SwerveModulePosition(position - steerCouplingDriveOffset, new Rotation2d(angle));
    }

    public Translation2d getGroundForceVector(double maxFrictionForce) {
        final double motorTorque = DCMotor.getKrakenX60Foc(1).getTorque(driveMotor.getTorqueCurrent());
        final double groundForce = Math.abs(motorTorque) * driveMotor.getMechanismRatio().reduction() / (wheelCircumference / (2.0 * Math.PI));
        return new Translation2d(MathUtility.clamp(groundForce, -maxFrictionForce, maxFrictionForce), new Rotation2d(getSteeringAngle() + (motorTorque < 0.0 ? Math.PI : 0.0)));
    }

    // --- BEGIN STUFF FOR SIMULATION ---

    private Timer m_simTimer;
    private SingleJointedArmSim m_steeringSim;

    /** Simulate one module with naive physics model. */
    public void updateSimSubtick(double velocity) {
        final double dt = m_simTimer.get();
        m_simTimer.reset();

        // Set drive speed from MinoSwerve Simulation
        driveMotor.setSimulatedSensorVelocity(velocity, dt, driveRatio);

        // Simulate steering
        m_steeringSim.setInput( steeringMotor.getPercentOutput() * RobotController.getBatteryVoltage());
        m_steeringSim.update(dt);
        steeringMotor.setSimulatedSensorPositionAndVelocity(m_steeringSim.getAngleRads(), m_steeringSim.getVelocityRadPerSec(), dt, steeringRatio);
    }

    // --- END STUFF FOR SIMULATION ---
}