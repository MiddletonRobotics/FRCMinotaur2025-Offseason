package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.LoggedTunableNumber;
import frc.robot.utilities.constants.GlobalConstants;
import frc.robot.utilities.constants.GlobalConstants.Mode;
import frc.robot.utilities.EqualsUtility;
import frc.robot.utilities.LoggedTracer;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drivetrain.CoastRequest;
import frc.robot.utilities.AlertManager;
import frc.robot.utilities.SwerveSetpointGenerator;
import frc.robot.utilities.SwerveSetpoint;

import lombok.Setter;

public class Drivetrain extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    private final Debouncer gyroConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private static final LoggedTunableNumber coastWaitTime = new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
    private static final LoggedTunableNumber coastMetersPerSecondThreshold = new LoggedTunableNumber("Drive/CoastMetersPerSecThreshold", .05);

    private final Timer lastMovementTimer = new Timer();

    private SwerveDriveKinematics kinematics = DrivetrainConstants.kDriveKinematics;
    private Rotation2d rawGyroRotation = new Rotation2d();

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] { // delta tracking
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(),
        new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
        }
    );

    private final SwerveSetpointGenerator swerveSetpointGenerator;

    @AutoLogOutput private boolean velocityMode = false;
    @AutoLogOutput private boolean brakeModeEnabled = true;

    private final SwerveDrivePoseEstimator poseEstimator;

    public enum CoastRequest {
        AUTOMATIC,
        ALWAYS_BRAKE,
        ALWAYS_COAST
    }
    
    @Setter @AutoLogOutput private CoastRequest coastRequest = CoastRequest.ALWAYS_BRAKE;

    public Drivetrain(GyroIO gyroIO, ModuleIO flModule, ModuleIO frModule, ModuleIO blModule, ModuleIO brModule) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModule, 0);
        modules[1] = new Module(frModule, 1);
        modules[2] = new Module(blModule, 2);
        modules[3] = new Module(brModule, 3);

        lastMovementTimer.start();
        setBrakeMode(true);

        swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics, DrivetrainConstants.kModuleTranslations);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
        PhoenixOdometryThread.getInstance().start();
    }

    @Override 
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drivetrain/Gyro", gyroInputs);

        for(Module module : modules) {
            module.updateInputs();
        }

        odometryLock.unlock();
        LoggedTracer.record("Drivetrain/Inputs");

        for(Module module : modules) {
            module.periodic();
        }

        if(DriverStation.isDisabled()) {
            for(Module module : modules) {
                module.stop();
            }

            Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("Drivetrain/SwerveStates/SetpointsUnoptimized", new SwerveModuleState[] {});
        }

        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle
                );

                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        if (Arrays.stream(modules).anyMatch((module) ->Math.abs(module.getVelocityMetersPerSecond()) > coastMetersPerSecondThreshold.get())) {
            lastMovementTimer.reset();
        }

        if (DriverStation.isEnabled()) {
            coastRequest = CoastRequest.ALWAYS_BRAKE;
        }

        switch (coastRequest) {
            case AUTOMATIC -> {
                if (DriverStation.isEnabled()) {
                    setBrakeMode(true);
                } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
                    setBrakeMode(false);
                }
            }

            case ALWAYS_BRAKE -> {
                setBrakeMode(true);
            }

            case ALWAYS_COAST -> {
                setBrakeMode(false);
            }
        }
      
        if (!velocityMode) {
            currentSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
        }

        gyroDisconnectedAlert.set(!gyroConnectedDebouncer.calculate(gyroInputs.connected) && GlobalConstants.kCurrentMode != Mode.SIM && !Robot.isJITing());
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled != enabled) {
          Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
        }

        brakeModeEnabled = enabled;
    }

    public void runVelocity(ChassisSpeeds speeds) {
        velocityMode = true;

        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);

        currentSetpoint = swerveSetpointGenerator.generateSetpoint(DrivetrainConstants.kModuleLimitsFree, currentSetpoint, discreteSpeeds, 0.02);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drivetrain/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
        Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drivetrain/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }
    }

    /**
     * Runs the drive at the desired velocity with setpoint module forces.
     *
     * @param speeds Speeds in meters/sec
     * @param moduleForces The forces applied to each module
     */
    public void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {
        velocityMode = true;

        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
        currentSetpoint = swerveSetpointGenerator.generateSetpoint(DrivetrainConstants.kModuleLimitsFree, currentSetpoint, discreteSpeeds, 0.02);
        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("Drivetrain/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
        Logger.recordOutput("Drivetrain/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drivetrain/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        // Save module forces to swerve states for logging
        SwerveModuleState[] wheelForces = new SwerveModuleState[4];

        // Send setpoints to modules
        SwerveModuleState[] moduleStates = getModuleStates();
        for (int i = 0; i < 4; i++) {
            // Optimize state
            Rotation2d wheelAngle = moduleStates[i].angle;
            setpointStates[i].optimize(wheelAngle);
            setpointStates[i].cosineScale(wheelAngle);

            // Calculate wheel torque in direction
            var wheelForce = moduleForces.get(i);
            Vector<N2> wheelDirection = VecBuilder.fill(wheelAngle.getCos(), wheelAngle.getSin());
            double wheelTorqueNm = wheelForce.dot(wheelDirection) * DrivetrainConstants.kWheelRadius.in(Meters);
            modules[i].runSetpoint(setpointStates[i], wheelTorqueNm);

            // Save to array for logging
            wheelForces[i] = new SwerveModuleState(wheelTorqueNm, setpointStates[i].angle);
        }

        Logger.recordOutput("Drivetrain/SwerveStates/ModuleForces", wheelForces);
    }

    public void runCharacterization(double output) {
        velocityMode = false;

        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = DrivetrainConstants.kModuleTranslations[i].getAngle();
        }

        kinematics.resetHeadings(headings);

        // Bypass swerve setpoint generator
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
        for (int i = 0; i < 4; i++) {
            states[i].optimize(modules[i].getAngle());
            modules[i].runSetpoint(states[i]);
        }
    }

    @AutoLogOutput(key = "Drivetrain/SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
          states[i] = modules[i].getPosition();
        }

        return states;
    }

    @AutoLogOutput(key = "Drivetrain/SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }

        return values;
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }

        return output;
    }

    public Rotation2d getGyroRotation() {
        return gyroInputs.yawPosition;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }
}