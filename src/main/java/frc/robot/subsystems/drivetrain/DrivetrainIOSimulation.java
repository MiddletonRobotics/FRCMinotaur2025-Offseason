package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.minolib.swerve.MapleSimulatedSwerveDrivetrain;
import frc.robot.constants.GlobalConstants;

public class DrivetrainIOSimulation extends DrivetrainIOCTRE {

    private static final double kSimulationLoopPeriod = 0.005; // 5 ms
    private Notifier simulatioNotifier = null;
    private double lastSimulationTime;
    public MapleSimulatedSwerveDrivetrain mapleSimulatedSwerveDrivetrain = null;

    Pose2d lastConsumedPose = null;

    public DrivetrainIOSimulation(SwerveDrivetrainConstants driveTrainConstants, @SuppressWarnings("rawtypes") SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        updateSimulationState();
    }

    @SuppressWarnings("unchecked")
    @Override
    public void updateSimulationState() {
        if (GlobalConstants.kUseMapleSim) {
            mapleSimulatedSwerveDrivetrain = new MapleSimulatedSwerveDrivetrain(
                Seconds.of(kSimulationLoopPeriod),
                Pounds.of(140),
                Inches.of(34),
                Inches.of(34),
                DCMotor.getKrakenX60(1),
                DCMotor.getKrakenX60(1),
                1.2,
                super.getModuleLocations(),
                super.getPigeon2(),
                super.getModules(),
                CompetitionTunerConstants.FrontLeft,
                CompetitionTunerConstants.FrontRight,
                CompetitionTunerConstants.BackLeft,
                CompetitionTunerConstants.BackRight
            );

            simulatioNotifier = new Notifier(mapleSimulatedSwerveDrivetrain::update);
        } else {
            lastSimulationTime = Utils.getCurrentTimeSeconds();
            simulatioNotifier = new Notifier(() -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimulationTime;
                lastSimulationTime = currentTime;
                updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
        }

        simulatioNotifier.startPeriodic(kSimulationLoopPeriod);
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        if (GlobalConstants.kUseMapleSim && mapleSimulatedSwerveDrivetrain != null) {
            mapleSimulatedSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
            Timer.delay(0.05);
        }
        super.resetOdometry(pose);
    }

    @Override
    public void updateDrivetrainInputs(DrivetrainIOInputs inputs) {
        inputs.Pose = getMapleSimulatedDrivetrain().getSimulatedDriveTrainPose();
        super.updateDrivetrainInputs(inputs);
    }

    public SwerveDriveSimulation getMapleSimulatedDrivetrain() {
        return mapleSimulatedSwerveDrivetrain.mapleSimDrive;
    }
}