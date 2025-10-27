package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.swerve.MapleSimSwerveDrivetrain;
import frc.minolib.swerve.PathPlannerLogging;
import frc.minolib.utilities.SubsystemDataProcessor;
import frc.minolib.vision.VisionPoseEstimate;
import frc.minolib.wpilib.RobotTime;
import frc.robot.constants.GlobalConstants;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class DrivetrainSubsystem extends SubsystemBase {
    DrivetrainIO io;

    DrivetrainIOInputsAutoLogged inputs = new DrivetrainIOInputsAutoLogged();
    ModuleIOInputsAutoLogged frontLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged frontRightInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backLeftInputs = new ModuleIOInputsAutoLogged();
    ModuleIOInputsAutoLogged backRightInputs = new ModuleIOInputsAutoLogged();

    private final Object moduleIOLock = new Object();

    private final ApplyRobotSpeeds stopRequest = new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final ApplyRobotSpeeds pathplannerAutoRequest = new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity).withDesaturateWheelSpeeds(true);

    private RobotConfig robotConfiguration;

    public DrivetrainSubsystem(DrivetrainIO io) {
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (moduleIOLock) {
                io.updateModuleInputs(frontLeftInputs, frontRightInputs, backLeftInputs, backRightInputs);
            }
        }, io);

        configurePathPlanner();
    }

    private ChassisSpeeds applyDeadband(ChassisSpeeds input) {
        if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) < 0.05) {
            input.vxMetersPerSecond = input.vyMetersPerSecond = 0.0;
        }

        if (Math.abs(input.omegaRadiansPerSecond) < 0.05) {
            input.omegaRadiansPerSecond = 0.0;
        }
        
        return input;
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
        io.updateDrivetrainInputs(inputs);

        synchronized (moduleIOLock) {
            Logger.processInputs("Drivetrain", inputs);
            Logger.processInputs("Drivetrain/Module Data/Front Left", frontLeftInputs);
            Logger.processInputs("Drivetrain/Module Data/Front Right", frontRightInputs);
            Logger.processInputs("Drivetrain/Module Data/Back Left", backLeftInputs);
            Logger.processInputs("Drivetrain/Module Data/Back Right", backRightInputs);
        }

        if (DriverStation.isDisabled()) {
            configureStandardDevsForDisabled();
        } else {
            configureStandardDevsForEnabled();
        }

        Logger.recordOutput("Drivetrain/LatencyPeriodicSeconds", RobotTime.getTimestampSeconds() - timestamp);
        Logger.recordOutput("Drivetrain/CurrentCommand", (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());

        configurePathPlanner();
    }

    private void configurePathPlanner() {
         try {
            robotConfiguration = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose
                this::resetOdometry,   // Consumer for seeding pose against auto
                () -> inputs.measuredChassisSpeeds, // Supplier of current robot speeds
                (speeds, feedforwards) -> applyRequest(() -> pathplannerAutoRequest.withSpeeds(speeds)),
                new PPHolonomicDriveController(new PIDConstants(2.5, 0, 0), new PIDConstants(4, 0, 0)), //rotation
                robotConfiguration,
                () -> !GlobalConstants.isBlueAlliance(), //path flips for red/blue alliance
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/targetPose", pose);
        });

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/currentPose", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("PathPlanner/activePath", activePath.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetChassisSpeedsCallback((chassisSpeeds) -> {
            Logger.recordOutput("PathPlanner/targetChassisSpeeds", chassisSpeeds);
        });
    }

    public void resetOdometry(Pose2d pose) {
        io.resetOdometry(pose);
    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
    }

    public void addVisionMeasurement(VisionPoseEstimate visionPoseEstimate) {
        io.addVisionMeasurement(visionPoseEstimate);
    }

    public void setStateStdDevs(double xStd, double yStd, double rotStd) {
        io.setStateStdDevs(xStd, yStd, rotStd);
    }

    public void configureStandardDevsForDisabled() {
        setStateStdDevs(1.0, 1.0, 1.0);
    }

    public void configureStandardDevsForEnabled() {
        setStateStdDevs(0.3, 0.3, 0.2);
    }

    public Pose2d getPose() {
        return inputs.Pose;
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        if (io instanceof DrivetrainIOSimulation) {
            return ((DrivetrainIOSimulation) io).getMapleSimDrive();
        }

        return null;
    }
}