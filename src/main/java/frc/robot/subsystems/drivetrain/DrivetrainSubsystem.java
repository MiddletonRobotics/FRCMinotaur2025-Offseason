package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.minolib.localization.VisionPoseEstimate;
import frc.minolib.swerve.pathplanner.PathPlannerLogging;
import frc.minolib.utilities.SubsystemDataProcessor;
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

    private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private final ApplyRobotSpeeds pathplannerAutoRequest = new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity).withDesaturateWheelSpeeds(true);

    public enum TeleopVelocityCoefficient {
        NORMAL(1, 0.9),
        SLOW(0.4, 0.6),
        ELEVATOR_RAISED(0.2, 0.3);

        private final double translationalVelocityCoefficient;
        private final double rotationalVelocityCoefficient;

        private TeleopVelocityCoefficient(double translationalVelocityCoefficient, double rotationalVelocityCoefficient) {
            this.translationalVelocityCoefficient = translationalVelocityCoefficient;
            this.rotationalVelocityCoefficient = rotationalVelocityCoefficient;
        }

        public double getTranslationalCoefficient() {
            return translationalVelocityCoefficient;
        }

        public double getRotationalCoefficient() {
            return rotationalVelocityCoefficient;
        }
    }

    private TeleopVelocityCoefficient teleopVelocityCoefficient = TeleopVelocityCoefficient.NORMAL;

    private RobotConfig robotConfiguration;

    public DrivetrainSubsystem(DrivetrainIO io) {
        this.io = io;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(() -> {
            synchronized (moduleIOLock) {
                io.updateModuleInputs(frontLeftInputs, frontRightInputs, backLeftInputs, backRightInputs);
            }
        }, io);

        //configurePathPlanner();
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
    }

    private void configurePathPlanner() {
         try {
            robotConfiguration = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                this::getPose,   // Supplier of current robot pose
                this::resetOdometry,   // Consumer for seeding pose against auto
                this::getRobotRelativeChassisSpeeds, // Supplier of current robot speeds
                (speeds, feedforwards) -> applyRobotChassisSpeeds(speeds, feedforwards.robotRelativeForcesX(), feedforwards.robotRelativeForcesY(), false),
                new PPHolonomicDriveController(new PIDConstants(2.5, 0, 0), new PIDConstants(4, 0, 0)), //rotation
                robotConfiguration,
                () -> !GlobalConstants.isBlueAlliance(), //path flips for red/blue alliance
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/TargetPose", pose);
        });

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            Logger.recordOutput("PathPlanner/CurrentPose", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("PathPlanner/ActivePath", activePath.toArray(new Pose2d[0]));
        });

        PathPlannerLogging.setLogTargetChassisSpeedsCallback((chassisSpeeds) -> {
            Logger.recordOutput("PathPlanner/TargetChassisSpeeds", chassisSpeeds);
        });
    }

    public Pose2d getPose() {
        return inputs.Pose;
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return inputs.measuredChassisSpeeds;
    }

    public void resetOdometry(Pose2d pose) {
        io.resetOdometry(pose);
    }

    public void setTargetChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        io.setTargetChassisSpeeds(chassisSpeeds);
    }

    public void setControl(SwerveRequest request) {
        io.setControl(request);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
    }

    public void applyRobotChassisSpeeds(ChassisSpeeds speeds, Force[] forcesX, Force[] forcesY, boolean isOpenLoop) {
        if (isOpenLoop) {
            setControl(this.pathplannerAutoRequest
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(forcesX)
                .withWheelForceFeedforwardsY(forcesY)
                .withCenterOfRotation(io.getCenterOfRotation())
            );
        } else {
            setControl(this.pathplannerAutoRequest
                .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(forcesX)
                .withWheelForceFeedforwardsY(forcesY)
                .withCenterOfRotation(io.getCenterOfRotation())
            );
        }
    }

    public void pointWheelTowards(Rotation2d targetDirection) {
        setControl(pointRequest.withModuleDirection(targetDirection));
    }

    public void applyXStance() {
        setControl(brakeRequest);
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

    public TeleopVelocityCoefficient getTeleopVelocityCoefficent() {
        return teleopVelocityCoefficient;
    }

    public void setTeleopVelocityCoefficient(TeleopVelocityCoefficient teleopVelocityCoefficient) {
        this.teleopVelocityCoefficient = teleopVelocityCoefficient;
    }

    public SwerveDriveSimulation getMapleSimDrive() {
        if (io instanceof DrivetrainIOSimulation) {
            return ((DrivetrainIOSimulation) io).getMapleSimulatedDrivetrain();
        }

        return null;
    }
}