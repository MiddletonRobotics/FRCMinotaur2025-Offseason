package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import frc.minolib.swerve.MapleSimSwerveDrivetrain;
import frc.robot.constants.GlobalConstants;

public class DrivetrainIOSimulation extends DrivetrainIOCTRE {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simNotifier = null;
    private double lastSimTime;
    public MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;

    Pose2d lastConsumedPose = null;

    public DrivetrainIOSimulation(SwerveDrivetrainConstants driveTrainConstants, @SuppressWarnings("rawtypes") SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        // Rewrite the telemetry consumer with a consumer for sim
        startSimThread();
    }

    @SuppressWarnings("unchecked")
    @Override
    public void registerTelemetryFunction(DrivetrainIOInputs inputs) {
        this.registerTelemetry(state -> {
            SwerveDriveState modifiedState = (SwerveDriveState) state;

            if(GlobalConstants.kUseMapleSim && mapleSimSwerveDrivetrain != null) {
                modifiedState.Pose = getMapleSimDrive().mapleSimDrive.getSimulatedDriveTrainPose();
            }

            inputs.logState(modifiedState);
        });
    }

    @SuppressWarnings("unchecked")
    public void startSimThread() {
        if (GlobalConstants.kUseMapleSim) {
            mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
                Seconds.of(kSimLoopPeriod),
                Pounds.of(140),
                Inches.of(34),
                Inches.of(34),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                1.2,
                getModuleLocations(),
                getPigeon2(),
                getModules(),
                CompetitionTunerConstants.FrontLeft,
                CompetitionTunerConstants.FrontRight,
                CompetitionTunerConstants.BackLeft,
                CompetitionTunerConstants.BackRight
            );

            simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        } else {
            lastSimTime = Utils.getCurrentTimeSeconds();
            simNotifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
        }

        simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void resetOdometry(Pose2d pose) {
        if (GlobalConstants.kUseMapleSim && mapleSimSwerveDrivetrain != null) {
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05);
        }
        super.resetOdometry(pose);
    }

    @Override
    public void updateDrivetrainInputs(DrivetrainIOInputs inputs) {
        super.updateDrivetrainInputs(inputs);
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return mapleSimSwerveDrivetrain;
    }
}