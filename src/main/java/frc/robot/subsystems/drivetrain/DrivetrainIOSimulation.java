package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.minolib.swerve.MapleSimSwerveDrivetrain;

public class DrivetrainIOSimulation extends DrivetrainIOCTRE implements Subsystem {
    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier simulatioNotifier = null;
    
    public DrivetrainIOSimulation(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
    
        startSimThread();

    }

    @Override
    public void updateInputs(DrivetrainIOInputsCollection inputs) {
        super.updateInputs(inputs);

        var pose = getPose();
        if (pose != null) {
            Logger.recordOutput("Drive/Viz/SimPose", getPose());
        }
    }

    public SwerveDriveSimulation getDrivetrainSimulation() {
        return mapleSimSwerveDrivetrain.mapleSimDrive;
    }

    @SuppressWarnings("unchecked")
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            Pounds.of(115), // robot weight
            Inches.of(34), // bumper length
            Inches.of(34), // bumper width
            DCMotor.getKrakenX60Foc(1), // drive motor type
            DCMotor.getKrakenX60Foc(1), // steer motor type
            1.2, // wheel COF
            super.getModuleLocations(),
            super.getPigeon2(),
            super.getModules(),
            CompetitionTunerConstants.FrontLeft,
            CompetitionTunerConstants.FrontRight,
            CompetitionTunerConstants.BackLeft,
            CompetitionTunerConstants.BackRight
        );

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        simulatioNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
        simulatioNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose() {
        return mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null) mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        
        Timer.delay(0.1); // wait for simulation to update
        super.resetPose(pose);
    }
}
