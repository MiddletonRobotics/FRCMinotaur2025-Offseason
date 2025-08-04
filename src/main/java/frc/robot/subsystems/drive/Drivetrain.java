package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Volts;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.utilities.PoseEstimator;
import frc.robot.utilities.PoseEstimator.TimestampedVisionUpdate;
import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.LoggedTunableNumber;
import frc.robot.utilities.constants.GlobalConstants;
import frc.robot.utilities.EqualsUtility;
import frc.robot.utilities.AlertManager;
import frc.robot.utilities.MeshedDrivingController;
import frc.robot.utilities.SubsystemProfiles;

public class Drivetrain extends SubsystemBase {
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    public final GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();

    private final Module[] modules = new Module[4];
    private final SysIdRoutine sysId;
    private Rotation2d lastGyroYaw = new Rotation2d();

    private Alert gyroDisconnectedAlert;

    public enum DriveProfiles {
        kDefault,
        kPathplanner,
        kAutoAlign,
        kDriveToPoint,
        kCharacterization,
        kMeshedUserControls,
        kStop
    }

    private SubsystemProfiles<DriveProfiles> driveProfiles;

    private ChassisSpeeds userChassisSpeeds;
    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();
    private ChassisSpeeds desiredAutoChassisSpeeds = null;

    private Rotation2d desiredHeading = new Rotation2d();

    private PIDController driveController;
    private PIDController autoDriveController;
    private PIDController headingController;

    private Pose2d driveToPointTargetPose = null;

    private Rotation2d rawGyroRotation = new Rotation2d();
        private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    
    private PoseEstimator poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));
    private MeshedDrivingController meshedController = new MeshedDrivingController(new Pose2d(), false, DrivetrainConstants.kDebounceAmount.get(), DrivetrainConstants.kMeshDrivePriority.get());

    public Drivetrain(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;
        this.gyroDisconnectedAlert = new Alert("Gyro Disconnected", Alert.AlertType.kError);

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        PhoenixOdometryThread.getInstance().start();

        driveController = new PIDController(
            DrivetrainConstants.kDriveToPointP.get(), 
            DrivetrainConstants.kDriveToPointI.get(), 
            DrivetrainConstants.kDriveToPointD.get()
        );

        autoDriveController = new PIDController(
            DrivetrainConstants.kDriveToPointAutoP.get(), 
            DrivetrainConstants.kDriveToPointAutoI.get(), 
            DrivetrainConstants.kDriveToPointAutoD.get()
        );

        headingController = new PIDController(
            DrivetrainConstants.kDriveToPointHeadingP.get(), 
            DrivetrainConstants.kDriveToPointHeadingI.get(), 
            DrivetrainConstants.kDriveToPointHeadingD.get()
        );


        sysId = new SysIdRoutine(new SysIdRoutine.Config(
            null, 
            null,
            null,
            (s) -> Logger.recordOutput("Drivetrain/SysIDState", s.toString())
        ), new SysIdRoutine.Mechanism(
            (volts) -> {
                for (int i = 0; i < modules.length; i++) {
                    modules[i].runCharacterization(volts.in(Volts));
                }
            },
            null,
            this
        ));

        HashMap<DriveProfiles, Runnable> periodicHash = new HashMap<>();
        periodicHash.put(DriveProfiles.kDefault, this::defaultPeriodic);
        periodicHash.put(DriveProfiles.kPathplanner, this::pathplannerPeriodic);
        periodicHash.put(DriveProfiles.kAutoAlign, this::autoAlignPeriodic);
        periodicHash.put(DriveProfiles.kDriveToPoint, this::driveToPointPeriodic);
        periodicHash.put(DriveProfiles.kCharacterization, this::characterizationPeriodic);
        periodicHash.put(DriveProfiles.kMeshedUserControls, this::meshedUserControlsPeriodic);
        periodicHash.put(DriveProfiles.kStop, this::stopPeriodic);

        driveProfiles = new SubsystemProfiles<>(periodicHash, DriveProfiles.kDefault);

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        driveController.setTolerance(Units.inchesToMeters(0.5));
        autoDriveController.setTolerance(Units.inchesToMeters(0.5));
        headingController.setTolerance(Units.degreesToRadians(1));

        AlertManager.registerAlert(gyroDisconnectedAlert);
    }
    
    @Override
    public void periodic() {
        double start = HALUtil.getFPGATime();
        double updateInputsStart = HALUtil.getFPGATime();

        odometryLock.lock();
        gyroIO.updateInputs(gyroIOInputs);

        for(Module module : modules) {
            module.updateInputs();
        }

        odometryLock.unlock();
        Logger.recordOutput("PeriodicTime/DrivetrainUpdateInputs", (HALUtil.getFPGATime() - updateInputsStart) / 1000.0);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            driveController.setP(DrivetrainConstants.kDriveToPointP.get());
            driveController.setI(DrivetrainConstants.kDriveToPointI.get());
            driveController.setD(DrivetrainConstants.kDriveToPointD.get());
        }, DrivetrainConstants.kDriveToPointP, DrivetrainConstants.kDriveToPointI, DrivetrainConstants.kDriveToPointD);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            autoDriveController.setP(DrivetrainConstants.kDriveToPointAutoP.get());
            autoDriveController.setI(DrivetrainConstants.kDriveToPointAutoI.get());
            autoDriveController.setD(DrivetrainConstants.kDriveToPointAutoD.get());
        }, DrivetrainConstants.kDriveToPointAutoP, DrivetrainConstants.kDriveToPointAutoI, DrivetrainConstants.kDriveToPointAutoD);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            headingController.setP(DrivetrainConstants.kDriveToPointHeadingP.get());
            headingController.setI(DrivetrainConstants.kDriveToPointHeadingI.get());
            headingController.setD(DrivetrainConstants.kDriveToPointHeadingD.get());
        }, DrivetrainConstants.kDriveToPointHeadingP, DrivetrainConstants.kDriveToPointHeadingI, DrivetrainConstants.kDriveToPointHeadingD);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            meshedController.setPIDControllers(
                DrivetrainConstants.kMeshedXYP.get(),
                DrivetrainConstants.kMeshedXYD.get(),
                DrivetrainConstants.kMeshedThetaP.get(),
                DrivetrainConstants.kMeshedThetaD.get() 
            );
        }, DrivetrainConstants.kMeshedXYP, DrivetrainConstants.kMeshedXYD, DrivetrainConstants.kMeshedThetaP, DrivetrainConstants.kMeshedThetaD);

        driveProfiles.getPeriodicFunctionTimed().run();

        double modulePeriodicStart = HALUtil.getFPGATime();
        Logger.processInputs("Drivetrain/Gyro", gyroIOInputs);

        for(Module module : modules) {
            module.periodic();
        }

        Logger.recordOutput("PeriodicTime/ModulePeriodic", (HALUtil.getFPGATime() - modulePeriodicStart) / 1000.0);

        if(DriverStation.isDisabled()) {
            for (Module module : modules) {
                module.stop();
            }

            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/setpointsOptimized", new SwerveModuleState[] {});
        }

        double odometryUpdateStart = HALUtil.getFPGATime();
        double[] sampleTimestamps = modules[0].getOdometryTimestamps();
        int sampleCount = sampleTimestamps.length;

        Twist2d totalTwist = new Twist2d();
        for(int i = 0; i < sampleCount; i++) {
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            
            for(int moduleIndex = 0; moduleIndex < modules.length; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle.minus(lastModulePositions[moduleIndex].angle)
                ); 
            }

            if(gyroIOInputs.connected) {
                rawGyroRotation = gyroIOInputs.odometryYawPosition[i];
            } else {
                Twist2d twist = DrivetrainConstants.kDriveKinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            Rotation2d deltaYaw;
            if(i == 0) {
                deltaYaw = rawGyroRotation.minus(lastGyroYaw);
            } else {
                deltaYaw = rawGyroRotation.minus(gyroIOInputs.odometryYawPosition[i - 1]);
            }

            Twist2d twist = DrivetrainConstants.kDriveKinematics.toTwist2d(moduleDeltas);
            twist = new Twist2d(twist.dx, twist.dy, deltaYaw.getRadians());
            totalTwist = new Twist2d(totalTwist.dx + twist.dx, totalTwist.dy + twist.dy, totalTwist.dtheta + twist.dtheta);
        }

        if(gyroIOInputs.connected) {
            totalTwist.dtheta = gyroIOInputs.yawPosition.minus(lastGyroYaw).getRadians();
            lastGyroYaw = gyroIOInputs.yawPosition;
        } else {
            totalTwist.dtheta = rawGyroRotation.minus(lastGyroYaw).getRadians();
            lastGyroYaw = rawGyroRotation;
        }

        poseEstimator.addDriveData(Timer.getTimestamp(), totalTwist);

        Logger.recordOutput("PeriodicTime/OdometryUpdate", (HALUtil.getFPGATime() - odometryUpdateStart) / 1000.0);
        Logger.recordOutput("Drivetrain/FreeFall", gyroIOInputs.zAcceleration < DrivetrainConstants.kFreefallAccelerationThreshold);
        Logger.recordOutput("Drivetrain/Profile", driveProfiles.getCurrentProfile());

        if(GlobalConstants.kUseAlerts && !gyroIOInputs.connected) {
            gyroDisconnectedAlert.set(true);
        } else {
            gyroDisconnectedAlert.set(false);
        }

        Logger.recordOutput("PeriodicTime/Drivetrain", (HALUtil.getFPGATime() - start) / 1000.0);
    }

    public void defaultPeriodic() {
        runVelocity(desiredChassisSpeeds);

        Logger.recordOutput("Drivetrain/DesiredHeading", desiredHeading.getDegrees());
        Logger.recordOutput("Drivetrain/CurrentHeading", getPose().getRotation().getDegrees());
        Logger.recordOutput("Drivetrain/DesiredSpeeds", desiredChassisSpeeds);
        Logger.recordOutput("Drivetrain/MeasureSpeeds", getChassisSpeeds());
    }

    public void characterizationPeriodic() {
        Logger.recordOutput("Drivetrain/DesiredHeading", desiredHeading.getDegrees());
        Logger.recordOutput("Drivetrain/CurrentHeading", getPose().getRotation().getDegrees());
        Logger.recordOutput("Drivetrain/DesiredSpeeds", desiredChassisSpeeds);
        Logger.recordOutput("Drivetrain/MeasureSpeeds", getChassisSpeeds());
    }

    public void pathplannerPeriodic() {
        desiredChassisSpeeds = desiredAutoChassisSpeeds;

        if(desiredAutoChassisSpeeds == null) {
            desiredChassisSpeeds = new ChassisSpeeds();
        }

        defaultPeriodic();
    }

    public void meshedUserControlsPeriodic() {
        desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            meshedController.calculateSpeeds(ChassisSpeeds.fromRobotRelativeSpeeds(userChassisSpeeds, getPose().getRotation()), getPose()), getPose().getRotation()
        );

        defaultPeriodic();
    }

    public void autoAlignPeriodic() {
        desiredChassisSpeeds = calculateAutoAlignSpeeds();
        defaultPeriodic();
    }

    public void stopPeriodic() {
        runVelocity(new ChassisSpeeds());
    }

    public void driveToPointPeriodic() {
        Pose2d currentPose = getPose();
        double currentDistance = currentPose.getTranslation().getDistance(driveToPointTargetPose.getTranslation());

        if(EqualsUtility.epsilonEquals(0, driveToPointTargetPose.getX())) {
            Logger.recordOutput("Drivetrain/ZeroTarget", Timer.getFPGATimestamp());
            desiredChassisSpeeds = new ChassisSpeeds();
            defaultPeriodic();

            return;
        }

        ChassisSpeeds currentSpeeds = getChassisSpeeds();
        double currentVelocity = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

        PIDController driveController = DriverStation.isAutonomous() ? autoDriveController : this.driveController;
        double pidVelocity = driveController.calculate(currentDistance, 0.0);

        if(currentDistance < driveController.getErrorTolerance()) {
            pidVelocity = 0.0;
        }

        double headingError = currentPose.getRotation().minus(driveToPointTargetPose.getRotation()).getRadians();
        double headingVelocity = headingController.calculate(currentPose.getRotation().getRadians(), driveToPointTargetPose.getRotation().getRadians());

        if(Math.abs(headingError) < headingController.getErrorTolerance()) {
            headingVelocity = 0.0;
        }

        pidVelocity = MathUtil.clamp(
            pidVelocity,
            -DrivetrainConstants.kMaxDriveToPointSpeed.in(MetersPerSecond),
            DrivetrainConstants.kMaxDriveToPointSpeed.in(MetersPerSecond)
        );

        Translation2d driveVelocity = new Pose2d(0.0, 0.0, currentPose.getTranslation().minus(driveToPointTargetPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(pidVelocity, 0.0, new Rotation2d())).getTranslation();

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveVelocity.getX(), driveVelocity.getY(), headingVelocity, currentPose.getRotation());
        desiredChassisSpeeds = speeds;

        Logger.recordOutput("DriveToPoint/TargetPose", driveToPointTargetPose);
        Logger.recordOutput("DriveToPoint/DriveDistance", currentDistance);
        Logger.recordOutput("DriveToPoint/HeadingError", headingError);
        Logger.recordOutput("DriveToPoint/TargetTheta", currentPose.getTranslation().minus(driveToPointTargetPose.getTranslation()).getAngle());

        Logger.recordOutput("DriveToPoint/PIDVelocity", pidVelocity);
        Logger.recordOutput("DriveToPoint/DriveVelocity", currentVelocity);
        Logger.recordOutput("DriveToPoint/HeadingVelocity", headingVelocity);
        Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
        Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());
        Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);

        if(!DriverStation.isAutonomous()) {
            if(driveToPointWithinTolerance()) {
                updateProfile(DriveProfiles.kDefault);
            }
        }

        defaultPeriodic();
    }

    public ChassisSpeeds calculateAutoAlignSpeeds() {
        if(desiredHeading != null) {
            Logger.recordOutput("AutoAlignHeading", desiredHeading);
            double output = headingController.calculate(getPose().getRotation().getRadians(), desiredHeading.getRadians());
            desiredChassisSpeeds.omegaRadiansPerSecond = output;
        }

        return desiredChassisSpeeds;
    }

    @SuppressWarnings("unused")
    public void runVelocity(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        if(RobotBase.isReal() && DrivetrainConstants.kRealReversed) {
            if(DriverStation.isTeleopEnabled()) {
                discreteSpeeds.vxMetersPerSecond = -discreteSpeeds.vxMetersPerSecond;
                discreteSpeeds.vyMetersPerSecond = -discreteSpeeds.vyMetersPerSecond;
            }

            discreteSpeeds.omegaRadiansPerSecond = -discreteSpeeds.omegaRadiansPerSecond;
        } else if(RobotBase.isSimulation() && DrivetrainConstants.kSimReversed) {
            discreteSpeeds.vxMetersPerSecond = -discreteSpeeds.vxMetersPerSecond;
            discreteSpeeds.vyMetersPerSecond = -discreteSpeeds.vyMetersPerSecond;
            discreteSpeeds.omegaRadiansPerSecond = -discreteSpeeds.omegaRadiansPerSecond;
        }

        SwerveModuleState[] setpointStates = DrivetrainConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        for(int i = 0; i < modules.length; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    }

    public void setDesiredChassisSpeeds(ChassisSpeeds speeds) {
        Logger.recordOutput("Drivetrain/Speed", speeds);
        desiredChassisSpeeds = speeds;
        userChassisSpeeds = speeds;
    }

    public void setAutoDesiredChassisSpeeds(ChassisSpeeds speeds) {
        desiredAutoChassisSpeeds = speeds;
    }

    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredChassisSpeeds;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DrivetrainConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void setDesiredheading(Rotation2d heading) {
        desiredHeading = heading;
    }

    public void setTargetPose(Pose2d pose) {
        driveToPointTargetPose = pose;
    }

    public Pose2d getTargetPose() {
        return driveToPointTargetPose;
    }

    public void stop() {
        updateProfile(DriveProfiles.kStop);
    }

    public void setCoast() {
        for(int i = 0; i < modules.length; i++) {
            modules[i].setBrakeMode(false);
        }
    }

    public void setBrake() {
        for(int i = 0; i < modules.length; i++) {
            modules[i].setBrakeMode(true);
        }
    }

    public void setDriveCoast() {
        for(int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrakeMode(false);
        }
    }

    public void setDriveBrake() {
        for(int i = 0; i < modules.length; i++) {
            modules[i].setDriveBrakeMode(true);
        }
    }

    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for(int i = 0; i < modules.length; i++) {
            headings[i] = DrivetrainConstants.kModuleTranslations[i].getAngle();
        }

        DrivetrainConstants.kDriveKinematics.resetHeadings(headings);
        stop();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for(int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }

        return states;
    }

    @SuppressWarnings("unused")
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for(int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }

        return positions;
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getLatestPose();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void addTimestampedVisionObservation(List<TimestampedVisionUpdate> observations) {
        poseEstimator.addVisionData(observations);
    }

    public void updateProfile(DriveProfiles profile) {
        driveProfiles.setCurrentProfile(profile);

        if(profile == DriveProfiles.kMeshedUserControls) {}
    }

    public boolean headingWithinTolerance() {
        return Math.abs(headingController.getError()) < Units.degreesToRadians(5);
    }

    public void setModuleCurrentLimits(double supplyLimit) {
        for(Module module : modules) {
            module.setCurrentLimits(supplyLimit);
        }
    }

    public DriveProfiles getCurrentProfile() {
        return driveProfiles.getCurrentProfile();
    }

    public void runCharacterization(double output) {
        if(driveProfiles.getCurrentProfile() != DriveProfiles.kCharacterization) {
            updateProfile(DriveProfiles.kCharacterization);
        }

        for(int i = 0; i < modules.length; i++) {
            modules[i].runCharacterization(output);
        }
    }

    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < modules.length; i++) {
            output += modules[i].getDriveVelocity() / 4.0;
        }

        return output;
    }

    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for(int i = 0; i < modules.length; i++) {
            values[i] = modules[i].getWheelRadiusCharacterization();
        }

        return values;
    }

    public Rotation2d getGyroRotation() {
        return rawGyroRotation;
    }

    public boolean driveToPointWithinTolerance() {
        return driveToPointWithinTolerance(null, null);
    }

    public boolean driveToPointWithinTolerance(Distance linearTolerance, Angle headingTolerance) {
        if(linearTolerance == null) {
            linearTolerance = Meters.of(DriverStation.isAutonomous() ? autoDriveController.getErrorTolerance() : driveController.getErrorTolerance());
        }

        if(headingTolerance == null) {
            headingTolerance = Rotation.of(headingController.getErrorTolerance());
        }

        return driveProfiles.getCurrentProfile() != DriveProfiles.kDriveToPoint ||
            (driveToPointTargetPose.getTranslation().getDistance(getPose().getTranslation()) < linearTolerance.in(Meters) &&
            driveToPointTargetPose.getRotation().minus(getPose().getRotation()).getMeasure().abs(Degrees) < headingTolerance.in(Degrees));
    }

    public double getYawVelocity() {
        return gyroIOInputs.yawVelocityRadiansPerSecond;
    }

    public double getMaxWheelSpeed() {
        double maxVelocity = 0.0;
        for(Module module : modules) {
            maxVelocity = Math.max(maxVelocity, Math.abs(module.getDriveVelocity()));
        }

        return maxVelocity;
    }

    public void setMeshedTargetPose(Pose2d pose) {
        meshedController.setDesiredPose(pose);
    }

    public void setMeshedAxisLocked(double slope, double intercept, double minX, double maxX) {
        meshedController.setAxisLocked(slope, intercept, minX, maxX);
    }

    public void setMeshedAxisUnlocked() {
        meshedController.setAxisUnlocked();
    }
}
