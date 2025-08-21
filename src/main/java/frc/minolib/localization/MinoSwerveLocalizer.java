package frc.minolib.localization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.minolib.wpilib.Fiducial;
import frc.minolib.wpilib.PoseEstimate;
import frc.minolib.vision.PipelineVisionPacket;
import frc.minolib.advantagekit.NTManager;
import frc.minolib.interfaces.MinoVisionCamera;
import frc.minolib.math.InterpolateableChassisSpeeds;
import frc.robot.utilities.AlignmentUtilities;
import frc.robot.utilities.Fiducials;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.TreeMap;
import java.util.concurrent.ConcurrentSkipListMap;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The QuixSwerveLocalizer class is responsible for managing the localization of a swerve drive
 * robot. It integrates odometry and vision measurements to provide an accurate estimate of the
 * robot's pose.
 *
 * <p>This class handles:
 *
 * <ul>
 *   <li>Sending and receiving data from pixelSigmas
 *   <li>Tracking AprilTags for vision-based localization
 *   <li>Maintaining buffers for odometry and chassis speeds
 *   <li>Updating the robot's pose using odometry and vision measurements
 *   <li>Publishing measurements and pose estimates to pixelSigmas
 * </ul>
 *
 * <p>Key components:
 *
 * <ul>
 *   <li>NTManager: Manages pixelSigmas communication
 *   <li>SwerveDriveOdometry: Handles odometry calculations
 *   <li>TimeInterpolatableBuffer: Buffers for interpolating odometry and chassis speeds
 *   <li>TreeMap: Stores measurements for processing
 * </ul>
 *
 * <p>Usage:
 *
 * <ul>
 *   <li>Initialize with kinematics, initial gyro angle, module positions, initial pose, targets,
 *       and cameras
 *   <li>Call update() method with odometry, vision packets, and chassis speeds to update the pose
 *   <li>Use getPose() to retrieve the latest pose estimate
 *   <li>Use resetPose() to reset the localizer to a specific pose
 * </ul>
 *
 * <p>Note: This class assumes that the robot is equipped with vision cameras and AprilTags for
 * localization.
 *
 * @param kinematics The kinematics of the swerve drive.
 * @param initialGyroAngle The initial angle of the gyro.
 * @param modulePositions The positions of the swerve modules.
 * @param priori The initial pose of the robot.
 * @param targets The fiducial targets for vision-based localization.
 * @param cameras The vision cameras used for detecting fiducial targets.
 */
public class MinoSwerveLocalizer {
    // Manages sending and receiving from pixelSigmas.
    private final NTManager m_pixelSigma = new NTManager();
    private final ArrayList<MinoVisionCamera> cameras;

    // If empty, uses all tags.
    private final HashSet<Integer> tagsToTrack = new HashSet<>();

    // ID of the current measurement. Used to sync between Robot and DriverStation.
    private int currentID = 0;
    // Map of {id: time}
    private final HashMap<Integer, Double> idToTimeMap = new HashMap<>();
    // Map of {time : SwerveDriveOdometryMeasurement}
    private final ConcurrentSkipListMap<Double, SwerveDriveOdometryMeasurement> timeToOdometryMap = new ConcurrentSkipListMap<>();
    // Buffer of poses so we can get the interpolated pose at the time of a vision measurement.
    private final double kBufferHistorySeconds = 10.0; // s
    private final TimeInterpolatableBuffer<Pose2d> rawOdometryPoseBuffer = TimeInterpolatableBuffer.createBuffer(kBufferHistorySeconds);
    // Buffer of chassis speeds so we can get the interpolated chassis speed at the time of a vision
    // measurement.
    private final TimeInterpolatableBuffer<InterpolateableChassisSpeeds> chassisSpeedsBuffer = TimeInterpolatableBuffer.createBuffer(kBufferHistorySeconds);
    // Map of {time: Pair<NTOdometryMeasurement, NTVisionMeasurement>}
    private final TreeMap<Double, Measurement> timeToMeasurementMap = new TreeMap<>();
    // ID of the last measurement that was updated.
    private int lastUpdatedID = -1;

    // Continuous odometry from the last reset. Used as input to the localizer.
    private final SwerveDriveOdometry rawOdometry;
    // Odometry played back on top of the latest localiation estimate.
    private final SwerveDriveOdometry playbackOdometry;
    // Odometry played back on top of the latest single-tag localiation estimate.
    private final SwerveDriveOdometry singleTagPlaybackOdometry;
    // Latest raw localization estimate from DS.
    private PoseEstimate latestRawEstimate = new PoseEstimate();

    // Measurements within |kMutableTimeBuffer| of the current time are not considered final.
    // This gives us a chance to associate new vision measurements with an past interpolated
    // odometry measurements.
    private final double kMutableTimeBuffer = 0.05; // seconds

    public MinoSwerveLocalizer(final SwerveDriveKinematics kinematics, final Rotation2d initialGyroAngle, final SwerveModulePosition[] modulePositions, final Pose2d priori, final Fiducial[] targets, final ArrayList<MinoVisionCamera> cameras) {
        rawOdometry = new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
        playbackOdometry = new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
        singleTagPlaybackOdometry = new SwerveDriveOdometry(kinematics, initialGyroAngle, modulePositions, priori);
        m_pixelSigma.publishTargets(targets);
        this.cameras = cameras;
        trackAllTags();
    }

    /** Resets the localizer to the given pose. */
    public void resetPose(
        final Rotation2d gyroAngle, final SwerveModulePosition[] modulePositions, final Pose2d pose) {
        rawOdometry.resetPosition(gyroAngle, modulePositions, pose);
        playbackOdometry.resetPosition(gyroAngle, modulePositions, pose);
        singleTagPlaybackOdometry.resetPosition(gyroAngle, modulePositions, pose);
    }

    public void trackAllTags() {
        tagsToTrack.clear();
        for (var tag : Fiducials.aprilTagFiducials) {
        tagsToTrack.add(tag.id());
        }
    }

    public void setTagsToTrack(int[] tagIDs) {
        tagsToTrack.clear();
        for (int id : tagIDs) {
        tagsToTrack.add(id);
        }
    }

    /** Raw odometry pose. */
    public Pose2d getOdometryPose() {
        return rawOdometry.getPoseMeters();
    }

    /** Localizer latency-compensated pose. */
    public Pose2d getPose() {
        return playbackOdometry.getPoseMeters();
    }

    /** Single tag latency-compensated pose. */
    public Pose2d getSingleTagPose() {
        return singleTagPlaybackOdometry.getPoseMeters();
    }

    /** Localizer pose from DS. Use for plotting/debugging only. */
    public Pose2d getRawPose() {
        return latestRawEstimate.getPose();
    }

    /** Update with odometry and optional vision. */
    public void update(
        final SwerveDriveOdometryMeasurement odometry,
        final ArrayList<PipelineVisionPacket> visionPackets,
        final ChassisSpeeds chassisSpeeds) {
        m_pixelSigma.publishCameras(cameras);

        final double startTimestamp = Timer.getFPGATimestamp();
        final double currentTime = Timer.getTimestamp();

        rawOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
        playbackOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
        singleTagPlaybackOdometry.update(odometry.getGyroAngle(), odometry.getModulePositionStates());
        timeToOdometryMap.put(currentTime, odometry);

        final var curPose = rawOdometry.getPoseMeters();
        rawOdometryPoseBuffer.addSample(currentTime, curPose);
        chassisSpeedsBuffer.addSample(currentTime, InterpolateableChassisSpeeds.fromChassisSpeeds(chassisSpeeds));

        // Always save latest odometry.
        timeToMeasurementMap.put(currentTime, new Measurement(curPose));

        // Save data from each camera.
        for (int cameraID = 0; cameraID < visionPackets.size(); cameraID++) {
            final ArrayList<Translation3d> detectedTags = new ArrayList<>();
            final var vision = visionPackets.get(cameraID);

            final double measurementTime = vision.getCaptureTimestamp();
            if (measurementTime > 0.0) {
                Logger.recordOutput("Localizer/measurementLatency[" + cameraID + "]", currentTime - measurementTime);
            }

            if (!vision.hasTargets()) {
                Translation3d[] array = new Translation3d[0];
                Logger.recordOutput("Localizer/detectedTags[" + cameraID + "]", array);
                continue;
            }

            // Merge with the existing the measurement if it already exists.
            Measurement existingMeasurement = timeToMeasurementMap.get(measurementTime);

            // If there is no existing measurement, create a new one by interpolating pose.
            if (existingMeasurement == null) {
                final Pose2d interpolatedPose = rawOdometryPoseBuffer.getSample(measurementTime).get();
                existingMeasurement = new Measurement(interpolatedPose);
                timeToMeasurementMap.put(measurementTime, existingMeasurement);
            }

            // Set vision uncertainty based on chassis speeds.
            // The fast we are moving, the more uncertain we are.
            // TODO: Tune
            final var interpolatedChassisSpeeds = chassisSpeedsBuffer.getSample(measurementTime).get();
            final double pixelSigma =
                Math.max(
                    100.0,
                    5.0
                        + 10.0
                            * Math.sqrt(
                                interpolatedChassisSpeeds.vxMetersPerSecond
                                        * interpolatedChassisSpeeds.vxMetersPerSecond
                                    + interpolatedChassisSpeeds.vyMetersPerSecond
                                        * interpolatedChassisSpeeds.vyMetersPerSecond)
                        + 20.0 * Math.abs(interpolatedChassisSpeeds.omegaRadiansPerSecond));
            existingMeasurement.setVisionUncertainty(pixelSigma);

            for (final var target : vision.getTargets()) {
                if (tagsToTrack.contains(target.getFiducialId())) {
                // Use AprilTag corners.
                for (int cornerID = 0; cornerID < target.getDetectedCorners().size(); cornerID++) {
                    existingMeasurement.addVisionMeasurement(
                        cameraID,
                        target.getFiducialId(),
                        cornerID,
                        target.getDetectedCorners().get(cornerID));
                }
                if (target.getFiducialId() <= Fiducials.aprilTagFiducials.length) {
                    detectedTags.add(
                        new Pose3d(getPose())
                            .transformBy(cameras.get(cameraID).getTransform())
                            .getTranslation());
                    detectedTags.add(
                        Fiducials.aprilTagFiducials[target.getFiducialId() - 1].getPose().getTranslation());
                }
                }
            }
            Translation3d[] array = new Translation3d[detectedTags.size()];
            detectedTags.toArray(array);
            Logger.recordOutput("Localizer/detectedTags[" + cameraID + "]", array);
        }
        
        publishImmutableEntries();
        final double endTimestamp = Timer.getFPGATimestamp();
        Logger.recordOutput("Localizer/UpdateMs", (endTimestamp - startTimestamp) * 1000.0);

        computeSingleTagPose();
    }

    // Do single-tag 3D-distance + angle based localization
    // Based on
    // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/c2bb0e79c466be33074577d51243258ec3d39f44/src/main/java/org/littletonrobotics/frc2025/RobotState.java#L194
    // TODO: This is semi-hardcoded for 2025
    private void computeSingleTagPose() {
        if (DriverStation.getAlliance().isEmpty()) {
            return;
        }

        final int tagID = AlignmentUtilities.determineClosestTagID(getPose(), DriverStation.getAlliance().get() == Alliance.Blue);

        // Get latest measurement from either camera with a target.
        double latestTimestamp = 0.0;
        PhotonTrackedTarget latestTarget = null;
        MinoVisionCamera cam = null;
        for (final var camera : cameras) {
            final var measurement = camera.getLatestMeasurement();
            if (measurement.getCaptureTimestamp() <= latestTimestamp) {
                continue;
            }

            for (final var target : measurement.getTargets()) {
                if (target.fiducialId != tagID) {
                    continue;
                }

                latestTimestamp = measurement.getCaptureTimestamp();
                latestTarget = target;
                cam = camera;
            }
        }

        if (latestTarget == null) {
            return;
        }

        // Compute single tag pose
        final Pose2d interpolatedPose = rawOdometryPoseBuffer.getSample(latestTimestamp).get();
        final Rotation2d interpolatedRotation = interpolatedPose.getRotation();
        final double distance = latestTarget.bestCameraToTarget.getTranslation().getNorm();
        final Transform3d robotToCam = cam.getTransform();
        final Translation2d camToTagTranslation =
            new Pose3d(
                    Translation3d.kZero,
                    new Rotation3d(
                        0,
                        Math.toRadians(-latestTarget.getPitch()),
                        Math.toRadians(-latestTarget.getYaw())))
                .transformBy(new Transform3d(new Translation3d(distance, 0, 0), Rotation3d.kZero))
                .getTranslation()
                .rotateBy(
                    new Rotation3d(robotToCam.getRotation().getX(), robotToCam.getRotation().getY(), 0))
                .toTranslation2d();
        final Rotation2d camToTagRotation =
            interpolatedRotation.plus(
                robotToCam.getRotation().toRotation2d().plus(camToTagTranslation.getAngle()));

        final Pose2d tagPose2d =
            Fiducials.aprilTagFiducials[latestTarget.fiducialId - 1].getPose().toPose2d();
        if (tagPose2d == null) {
        return;
        }

        final Translation2d fieldToCameraTranslation =
            new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(new Transform2d(camToTagTranslation.getNorm(), 0.0, Rotation2d.kZero))
                .getTranslation();
        final Pose3d cameraPose = Pose3d.kZero.transformBy(robotToCam);
        Pose2d robotPose =
            new Pose2d(
                    fieldToCameraTranslation,
                    interpolatedRotation.plus(cameraPose.toPose2d().getRotation()))
                .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));
        // Use gyro angle at time for robot rotation
        robotPose = new Pose2d(robotPose.getTranslation(), interpolatedRotation);

        Logger.recordOutput("Swerve/SingleTagPoseRaw", robotPose);

        // Replay odometry on top of latest estimate
        Double curTime = timeToOdometryMap.ceilingKey(latestTimestamp);
        if (curTime == null) {
        return;
        }
        final var measurement = timeToOdometryMap.get(curTime);
        singleTagPlaybackOdometry.resetPosition(
            measurement.getGyroAngle(), measurement.getModulePositionStates(), robotPose);

        // Traverse entries in |timeToOdometryMap| from |curTime| until the end to update
        // playback odometry.
        while (curTime != null) {
        final SwerveDriveOdometryMeasurement lastMeasurment = timeToOdometryMap.get(curTime);
        singleTagPlaybackOdometry.update(
            lastMeasurment.getGyroAngle(), lastMeasurment.getModulePositionStates());
        curTime = timeToOdometryMap.higherKey(curTime);
        }
    }

    /**
     * Uses the latest pose estimate over pixelSigmas and replays the latest odometry on top of it.
     */
    public void updateWithLatestPoseEstimate() {
        final double startTimestamp = Timer.getFPGATimestamp();
        m_pixelSigma.updateInputs();
        final PoseEstimate estimate = m_pixelSigma.getLatestPoseEstimate();

        // Save for plotting/debugging purposes.
        latestRawEstimate = estimate;

        // Only incorporate estimate if it is new.
        if (idToTimeMap.size() == 0 || estimate.getID() == lastUpdatedID) {
        final double endTimestamp = Timer.getFPGATimestamp();
        Logger.recordOutput(
            "Localizer/UpdateWithLatestPoseEstimateMs", (endTimestamp - startTimestamp) * 1000.0);
        Logger.recordOutput("Localizer/visionCorrection (m)", 0.0);
        return;
        }
        lastUpdatedID = estimate.getID();

        // Save the pose before correction.
        final Pose2d preCorrectionPose = getPose();

        // Start playback odometry at the first time >= the current estimate.
        final double estimateTime = idToTimeMap.get(estimate.getID());
        Double curTime = timeToOdometryMap.ceilingKey(estimateTime);

        final var measurement = timeToOdometryMap.get(curTime);
        playbackOdometry.resetPosition(
            measurement.getGyroAngle(), measurement.getModulePositionStates(), estimate.getPose());

        // Traverse entries in |timeToOdometryMap| from |curTime| until the end to update
        // playback odometry.
        while (curTime != null) {
        final SwerveDriveOdometryMeasurement lastMeasurment = timeToOdometryMap.get(curTime);
        playbackOdometry.update(
            lastMeasurment.getGyroAngle(), lastMeasurment.getModulePositionStates());
        curTime = timeToOdometryMap.higherKey(curTime);
        }
        final double endTimestamp = Timer.getFPGATimestamp();
        Logger.recordOutput(
            "Localizer/UpdateWithLatestPoseEstimateMs", (endTimestamp - startTimestamp) * 1000.0);

        // Log magnitude of correction
        final Pose2d postCorrectionPose = getPose();
        Logger.recordOutput(
            "Localizer/visionCorrection (m)",
            postCorrectionPose.minus(preCorrectionPose).getTranslation().getNorm());
    }

    /** Handles NT publishing, ID finalization, and cleanup. */
    private void publishImmutableEntries() {
        final double currentTime = Timer.getTimestamp();

        // Times are in ascending order.
        final ArrayList<Double> times = new ArrayList<>(timeToMeasurementMap.keySet());
        for (final double time : times) {
        // Entries within |kMutableTimeBuffer| of the current time are not considered final.
        // Once we reach this point we are done.
        if (currentTime - time < kMutableTimeBuffer) {
            break;
        }

        // Entries older than |kMutableTimeBuffer| are considered immutable.
        // Assign them an ID and publish them.
        final var measurement = timeToMeasurementMap.get(time);
        m_pixelSigma.publishMeasurement(measurement, currentID);
        timeToMeasurementMap.remove(time);

        idToTimeMap.put(currentID, time);
        currentID += 1;
        }
    }
}