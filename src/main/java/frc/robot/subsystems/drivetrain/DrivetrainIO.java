package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.minolib.vision.VisionPoseEstimate;

public interface DrivetrainIO extends SubsystemDataProcessor.IODataRefresher {
    @AutoLog
    public class DrivetrainIOInputs {
        public Pose2d Pose = new Pose2d();
        public ChassisSpeeds referenceChassisSpeeds = new ChassisSpeeds();
        public ChassisSpeeds measuredChassisSpeeds = new ChassisSpeeds();
        public SwerveModuleState[] targetModuleStates;
        public SwerveModuleState[] referenceModuleStates;
        public SwerveModulePosition[] modulePositions;
        public Rotation2d rawHeading = new Rotation2d();
        public double timestamp;
        public double odometryPeriod;
        public int successfulDaqs;
        public int failedDaqs;

        public void logState(SwerveDrivetrain.SwerveDriveState state) {
            this.Pose = state.Pose;
            this.measuredChassisSpeeds = state.Speeds;
            this.targetModuleStates = state.ModuleTargets;
            this.referenceModuleStates = state.ModuleStates;
            this.modulePositions = state.ModulePositions;
            this.rawHeading = state.RawHeading;
            this.timestamp = state.Timestamp;
            this.odometryPeriod = state.OdometryPeriod;
            this.successfulDaqs = state.SuccessfulDaqs;
            this.failedDaqs = state.FailedDaqs;
        }
    }

    @AutoLog
    public class ModuleIOInputs {
        public boolean isDriveMotorConnected = false;
        public double driveSupplyCurrentAmperes = 0.0;
        public double driveStatorCurrentAmperes = 0.0;
        public double driveAppliedVoltage = 0.0;
        public double driveTemperatureCelsius = 0.0;

        public boolean isSteerMotorConnected = false;
        public double steerSupplyCurrentAmperes = 0.0;
        public double steerStatorCurrentAmperes = 0.0;
        public double steerAppliedVoltage = 0.0;
        public double steerTemperatureCelsius = 0.0;
    }

    public void registerTelemetryFunction(DrivetrainIOInputs inputs);

    public void updateDrivetrainInputs(DrivetrainIOInputs inputs);

    public void updateModuleInputs(ModuleIOInputs... inputs);

    public void setCenterOfRotation(Translation2d centerOfRotation);

    public Translation2d getCenterOfRotation();

    public void setControl(SwerveRequest request);

    public void setBrakeMode(boolean enable);

    public void setTargetChassisSpeeds(ChassisSpeeds targetChassisSpeeds);

    public default void updateSimulationState() {};

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier, Subsystem subsystemRequired);

    public void setStateStdDevs(double xStd, double yStd, double rotStd);

    public void addVisionMeasurement(VisionPoseEstimate visionPoseEstimate);

    public void resetOdometry(Pose2d pose);

    public void refreshData();
}
