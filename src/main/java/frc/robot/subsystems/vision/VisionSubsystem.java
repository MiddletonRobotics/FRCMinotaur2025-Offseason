package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.minolib.RobotConfiguration;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.localization.MinoRobotOdometry;
import frc.minolib.vision.PhotonFiducialResult;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.config.RobotConfig;

/**
 * The Vision subsystem is responsible for updating the robot's estimated pose based on a collection
 * of cameras capturing AprilTags. The Vision subsystem is comprised of multiple VisionIO objects,
 * each of which is responsible for producing a single PhotonPipelineResult. There is a one-to-one
 * relationship between each VisionIO object and each co-processor (e.g., Raspberry Pi) running
 * PhotonVision.
 */
public class VisionSubsystem extends SubsystemBase {
  private VisionIO[] visionIOs;
  private final VisionIOInputsAutoLogged[] inputs;
  private double[] lastTimestamps;
  private int[] cyclesWithNoResults;
  private int[] updatePoseCount;
  private Alert[] disconnectedAlerts;

  private List<Integer> camerasToConsider = new ArrayList<>();

  private AprilTagFieldLayout layout;
  private Alert noAprilTagLayoutAlert = new Alert("No AprilTag layout file found. Update APRILTAG_FIELD_LAYOUT_PATH in VisionConstants.java", AlertType.kWarning);
  private final Alert unofficialAprilTagLayoutAlert = new Alert("", AlertType.kInfo);

  private boolean isEnabled = true;
  private boolean isVisionUpdating = false;

  private MinoRobotOdometry odometry;

  private Pose3d mostRecentBestPose = new Pose3d();
  private double mostRecentBestPoseTimestamp = 0.0;
  private double mostRecentBestPoseStdDev = 0.0;

  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
  private final Map<Pose3d, Double> lastPoseEstimationAcceptedTimes = new HashMap<>();
  private final Map<Pose3d, Double> lastPoseEstimationRejectedTimes = new HashMap<>();

  private List<Pose3d> allRobotPoses = new ArrayList<>();
  private List<Pose3d> allRobotPosesAccepted = new ArrayList<>();
  private List<Pose3d> allRobotPosesRejected = new ArrayList<>();
  private List<Pose3d> allTagPoses = new ArrayList<>();

  private List<List<Pose3d>> tagPoses;
  private List<List<Pose3d>> cameraPoses;
  private List<List<Pose3d>> robotPoses;
  private List<List<Pose3d>> robotPosesAccepted;
  private List<List<Pose3d>> robotPosesRejected;

  private final Pose3d[] robotPosesForCalibration;

  private final LoggedTunableNumber latencyAdjustmentSeconds = new LoggedTunableNumber("Vision/LatencyAdjustmentSeconds", 0.0);
  private final LoggedTunableNumber ambiguityScaleFactor = new LoggedTunableNumber("Vision/AmbiguityScaleFactor", VisionConstants.kAmbiguityScaleFactor);
  private final LoggedTunableNumber reprojectionErrorScaleFactor = new LoggedTunableNumber("Vision/ReprojectionErrorScaleFactor", VisionConstants.kReprojectionErrorScaleFactor);
  private final LoggedTunableNumber xyStdDevCoefficient = new LoggedTunableNumber("Vision/XYStdDevCoefficient", VisionConstants.linearStandardDeviation);
  private final LoggedTunableNumber thetaStdDevCoefficient = new LoggedTunableNumber("Vision/ThetaStdDevCoefficient", VisionConstants.thetaStandardDeviation);

  /**
   * Create a new Vision subsystem. The number of VisionIO objects passed to the constructor must
   * match the number of robot-to-camera transforms returned by the RobotConfig singleton.
   *
   * @param visionIO One or more VisionIO objects, each of which is responsible for producing a
   *     single single PhotonPipelineResult. There is a one-to-one relationship between each
   *     VisionIO object and each co-processor (e.g., Raspberry Pi) running PhotonVision.
   */
  public VisionSubsystem(VisionIO[] visionIOs) {
    this.visionIOs = visionIOs;
    this.lastTimestamps = new double[visionIOs.length];
    this.cyclesWithNoResults = new int[visionIOs.length];
    this.updatePoseCount = new int[visionIOs.length];
    this.inputs = new VisionIOInputsAutoLogged[visionIOs.length];
    this.disconnectedAlerts = new Alert[visionIOs.length];
    this.camerasToConsider = new ArrayList<>();

    tagPoses = new ArrayList<List<Pose3d>>(visionIOs.length);
    cameraPoses = new ArrayList<List<Pose3d>>(visionIOs.length);
    robotPoses = new ArrayList<List<Pose3d>>(visionIOs.length);
    robotPosesAccepted = new ArrayList<List<Pose3d>>(visionIOs.length);
    robotPosesRejected = new ArrayList<List<Pose3d>>(visionIOs.length);

    for (int i = 0; i < visionIOs.length; i++) {
      this.inputs[i] = new VisionIOInputsAutoLogged();
      this.disconnectedAlerts[i] = new Alert("camera" + i + " is disconnected", AlertType.kError);
      this.camerasToConsider.add(i);

      tagPoses.add(new ArrayList<>());
      cameraPoses.add(new ArrayList<>());
      robotPoses.add(new ArrayList<>());
      robotPosesAccepted.add(new ArrayList<>());
      robotPosesRejected.add(new ArrayList<>());
    }

    // retrieve a reference to the pose estimator singleton
    this.odometry = MinoRobotOdometry.getInstance();

    // load and log all of the AprilTags in the field layout file
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      noAprilTagLayoutAlert.set(false);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      noAprilTagLayoutAlert.set(true);
    }

    // AprilTag layout alert
    if (!VisionConstants.APRILTAG_FIELD_LAYOUT_PATH.equals(VisionConstants.OFFICIAL_APRILTAG_FIELD_LAYOUT_PATH)) {
      unofficialAprilTagLayoutAlert.set(true);
      unofficialAprilTagLayoutAlert.setText("Unofficial AprilTag layout in use (" + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH.toString() + ").");
    }

    Pose3d[] aprilTagsPoses = new Pose3d[this.layout.getTags().size()];
    for (int i = 0; i < aprilTagsPoses.length; i++) {
      aprilTagsPoses[i] = this.layout.getTags().get(i).pose;
    }
    Logger.recordOutput(VisionConstants.kSubsystemName + "/AprilTagsPoses", aprilTagsPoses);

    // robot to camera transformation calibration
    robotPosesForCalibration = new Pose3d[visionIOs.length];
    // FL and FR cameras are calibrated based on a centered AprilTag on the robot-to-camera
    // transform calibration jig
    robotPosesForCalibration[0] =
      new Pose3d(FieldConstants.Reef.centerFaces[0].transformBy(
        new Transform2d(RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0, 0, Rotation2d.fromDegrees(180))));
    robotPosesForCalibration[2] = robotPosesForCalibration[0];

    // the BR camera is calibrated based on an offset AprilTag on the robot-to-camera transform
    // calibration jig
    robotPosesForCalibration[1] = new Pose3d(FieldConstants.Reef.centerFaces[0]).transformBy(
      new Transform3d(
        RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
        Units.inchesToMeters(14.0),
        -Units.inchesToMeters(1.0),
        new Rotation3d()
      )
    );

    // the BL camera is calibrated based on an offset AprilTag on the robot-to-camera transform
    // calibration jig
    robotPosesForCalibration[3] = new Pose3d(FieldConstants.Reef.centerFaces[0]).transformBy(
      new Transform3d(
        RobotConfiguration.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
        -Units.inchesToMeters(14.0),
        -Units.inchesToMeters(1.0),
        new Rotation3d()
      )
    );
  }

  /**
   * This method is invoked each iteration of the scheduler. It updates the inputs for each of the
   * VisionIO objects and, for each, updates the pose estimator based on the most recent detected
   * AprilTags.
   */
  @Override
  public void periodic() {
    isVisionUpdating = false;

    for (int cameraIndex = 0; cameraIndex < visionIOs.length; cameraIndex++) {
      visionIOs[cameraIndex].updateInputs(inputs[cameraIndex]);
      Logger.processInputs(VisionConstants.kSubsystemName + "/" + cameraIndex, inputs[cameraIndex]);
    }

    this.allRobotPoses.clear();
    this.allRobotPosesAccepted.clear();
    this.allRobotPosesRejected.clear();
    this.allTagPoses.clear();

    for (int cameraIndex = 0; cameraIndex < visionIOs.length; cameraIndex++) {

      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].isCameraConnected);
      this.cyclesWithNoResults[cameraIndex] += 1;

      // Initialize logging values
      tagPoses.get(cameraIndex).clear();
      cameraPoses.get(cameraIndex).clear();
      robotPoses.get(cameraIndex).clear();
      robotPosesAccepted.get(cameraIndex).clear();
      robotPosesRejected.get(cameraIndex).clear();

      for (PhotonFiducialResult observation : inputs[cameraIndex].visionPacket) {
        // only process the vision data if the timestamp is newer than the last one
        if (this.lastTimestamps[cameraIndex] < observation.timestamp()) {

          if (VisionConstants.kCalibrateCameraTranformation) {
            logCameraTransforms(cameraIndex, observation);
          }

          // Initialize logging values
          this.lastTimestamps[cameraIndex] = observation.timestamp();
          cameraPoses.get(cameraIndex).add(observation.cameraPose());
          Pose3d estimatedRobotPose3d = observation.cameraPose().plus(
            RobotConfiguration.getInstance().getRobotToCameraTransforms()[cameraIndex].inverse()
          );

          Pose2d estimatedRobotPose2d = estimatedRobotPose3d.toPose2d();
          robotPoses.get(cameraIndex).add(estimatedRobotPose3d);

          // only update the pose estimator if the vision subsystem is enabled and the vision's
          // estimated pose is on (or very close to) the field
          // for multi-tag strategies, ensure the reprojection error is less than the threshold; for
          // single-tag, ensure the ambiguity is less than the threshold.
          boolean acceptPose = isEnabled
            && this.camerasToConsider.contains(cameraIndex)
            && (observation.type() == PoseObservationType.MULTI_TAG || observation.poseAmbiguity() < VisionConstants.kAmbiguityThreshold)
            && (observation.type() == PoseObservationType.SINGLE_TAG || Math.abs(observation.reprojectionError()) < VisionConstants.kReprojectionErrorThreshold)
            && poseIsOnField(estimatedRobotPose3d)
            && arePoseRotationsReasonable(estimatedRobotPose3d);

          if (acceptPose) {
            // get tag poses and update last detection times
            final int finalCameraIndex = cameraIndex;
            for (int tagID = 1; tagID < VisionConstants.MAX_NUMBER_TAGS; tagID++) {
              if ((observation.tagsSeenBitMap() & (1L << tagID)) != 0) {
                if (VisionConstants.kDetailedLoggnig) {
                  lastTagDetectionTimes.put(tagID, Timer.getTimestamp());
                }

                Optional<Pose3d> tagPose = this.layout.getTagPose(tagID);
                tagPose.ifPresent((e) -> {
                  tagPoses.get(finalCameraIndex).add(e);
                });
              }
            }

            robotPosesAccepted.get(cameraIndex).add(estimatedRobotPose3d);
            if (VisionConstants.kDetailedLoggnig) {
              lastPoseEstimationAcceptedTimes.put(estimatedRobotPose3d, Timer.getTimestamp());
            }

            Matrix<N3, N1> stdDev = getStandardDeviations(cameraIndex, observation);
            odometry.addVisionMeasurement(estimatedRobotPose2d, observation.timestamp(), latencyAdjustmentSeconds.get(), stdDev);

            // if the most-recent "best pose" is too old, capture a new one regardless of its
            // standard deviation values; otherwise, only capture a new one if its standard
            // deviation is lower than the current best pose
            if (mostRecentBestPoseTimestamp < Timer.getTimestamp() - VisionConstants.kBestPoseThreshold || mostRecentBestPoseStdDev > stdDev.get(0, 0)) {
              mostRecentBestPose = estimatedRobotPose3d;
              mostRecentBestPoseTimestamp = observation.timestamp();
              mostRecentBestPoseStdDev = stdDev.get(0, 0);
            }

            this.updatePoseCount[cameraIndex]++;

            // if there are multiple observations, only the last will be logged, which is fine
            Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/UpdatePoseCount", this.updatePoseCount[cameraIndex]);
            Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/StdDevX", stdDev.get(0, 0));
            Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/StdDevY", stdDev.get(1, 0));
            Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/StdDevT", stdDev.get(2, 0));
          } else {
            robotPosesRejected.get(cameraIndex).add(estimatedRobotPose3d);
            if (VisionConstants.kDetailedLoggnig) {
              lastPoseEstimationRejectedTimes.put(estimatedRobotPose3d, Timer.getTimestamp());
            }
          }

          this.cyclesWithNoResults[cameraIndex] = 0;
          isVisionUpdating = true;
        }
      }

      // Log data for this camera
      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/LatencySecs", Timer.getTimestamp() - this.lastTimestamps[cameraIndex]);
      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/CyclesWithNoResults", this.cyclesWithNoResults[cameraIndex]);
      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/TagPoses", tagPoses.get(cameraIndex).toArray(Pose3d[]::new));
      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/CameraPoses", cameraPoses.get(cameraIndex).toArray(new Pose3d[cameraPoses.get(cameraIndex).size()]));
      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/RobotPoses", robotPoses.get(cameraIndex).toArray(new Pose3d[robotPoses.get(cameraIndex).size()]));
      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/RobotPosesAccepted",
          robotPosesAccepted.get(cameraIndex).toArray(new Pose3d[robotPosesAccepted.get(cameraIndex).size()]));

      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/RobotPosesRejected",
          robotPosesRejected.get(cameraIndex).toArray(new Pose3d[robotPosesRejected.get(cameraIndex).size()]));

      Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/CameraAxes", new Pose3d(MinoRobotOdometry.getInstance().getEstimatedPose()).plus(RobotConfiguration.getInstance().getRobotToCameraTransforms()[cameraIndex]));

      if (!VisionConstants.kDetailedLoggnig) {
        allRobotPosesAccepted.addAll(robotPosesAccepted.get(cameraIndex));
        allRobotPosesRejected.addAll(robotPosesRejected.get(cameraIndex));
        allTagPoses.addAll(tagPoses.get(cameraIndex));
      }
    }

    // Log summary data
    if (VisionConstants.kDetailedLoggnig) {
      for (Map.Entry<Pose3d, Double> entry : lastPoseEstimationAcceptedTimes.entrySet()) {
        if (Timer.getTimestamp() - entry.getValue() < VisionConstants.kTagLogThreshold) {
          allRobotPosesAccepted.add(entry.getKey());
        }
      }
    }

    Logger.recordOutput(VisionConstants.kSubsystemName + "/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));

    if (VisionConstants.kDetailedLoggnig) {
      for (Map.Entry<Pose3d, Double> entry : lastPoseEstimationRejectedTimes.entrySet()) {
        if (Timer.getTimestamp() - entry.getValue() < VisionConstants.kTagLogThreshold) {
          allRobotPosesRejected.add(entry.getKey());
        }
      }
    }

    Logger.recordOutput(VisionConstants.kSubsystemName + "/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));

    allRobotPoses.addAll(allRobotPosesAccepted);
    allRobotPoses.addAll(allRobotPosesRejected);
    Logger.recordOutput(VisionConstants.kSubsystemName + "/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));

    // Log tag poses
    if (VisionConstants.kDetailedLoggnig) {
      for (Map.Entry<Integer, Double> detectionEntry : lastTagDetectionTimes.entrySet()) {
        if (Timer.getTimestamp() - detectionEntry.getValue() < VisionConstants.kTagLogThreshold) {
          layout.getTagPose(detectionEntry.getKey()).ifPresent(allTagPoses::add);
        }
      }
    }

    Logger.recordOutput(VisionConstants.kSubsystemName + "/AprilTags", allTagPoses.toArray(Pose3d[]::new));
    Logger.recordOutput(VisionConstants.kSubsystemName + "/IsEnabled", isEnabled);
    Logger.recordOutput(VisionConstants.kSubsystemName + "/IsUpdating", isVisionUpdating);
    Logger.recordOutput(VisionConstants.kSubsystemName + "/CamerasToConsider", camerasToConsider.toString());

    // Record cycle time
    LoggedTracer.record("Vision");
  }

  public void specifyCamerasToConsider(List<Integer> cameraIndices) {
    this.camerasToConsider = cameraIndices;
  }

  /**
   * Returns true if the vision subsystem is enabled.
   *
   * @return true if the vision subsystem is enabled
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  /**
   * Returns the estimated robot pose based on the most recent vision data. This method is used to
   * reset the robot's odometry based solely on the vision data.
   *
   * @return the estimated robot pose based on the most recent vision data
   */
  public Pose3d getBestRobotPose() {
    if (mostRecentBestPoseTimestamp > Timer.getTimestamp() - VisionConstants.kBestPoseThreshold) {
      return mostRecentBestPose;
    } else {
      return null;
    }
  }

  /**
   * Enable or disable the vision subsystem.
   *
   * @param enable enables the vision subsystem if true; disables if false
   */
  public void enable(boolean enable) {
    isEnabled = enable;
  }

  /**
   * Returns true if the specified pose is on the field (or within FIELD_BORDER_MARGIN_METERS in the
   * x and y directions and MAX_Z_ERROR_METERS in the z direction).
   *
   * @param pose the pose to check
   * @return true if the specified pose is on the field (or very close to it)
   */
  private boolean poseIsOnField(Pose3d pose) {
    return pose.getX() > -VisionConstants.kFieldBorderMarginError
        && pose.getX() < layout.getFieldLength() + VisionConstants.kFieldBorderMarginError
        && pose.getY() > -VisionConstants.kFieldBorderMarginError
        && pose.getY() < layout.getFieldWidth() + VisionConstants.kFieldBorderMarginError
        && Math.abs(pose.getZ()) < VisionConstants.maxZErrorMeters;
  }

  private boolean arePoseRotationsReasonable(Pose3d pose) {
    return Math.abs(MinoRobotOdometry.getInstance()
      .getEstimatedPose()
      .getRotation()
      .minus(pose.getRotation().toRotation2d())
      .getRadians()
    ) < VisionConstants.kRotationThreshold;
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   */
  private Matrix<N3, N1> getStandardDeviations(int index, PhotonFiducialResult observation) {
    double xyStdDev = xyStdDevCoefficient.get()
        * Math.pow(observation.averageDistance(), 2.0)
        / observation.numberOfFiducials()
        * RobotConfiguration.getInstance().getCameraStdDevFactors()[index];

    // for multi-tag strategies, scale the standard deviation by the reprojection error; for
    // single-tag, scale by the ambiguity

    if (observation.type() == PoseObservationType.MULTI_TAG) {
      xyStdDev *= (reprojectionErrorScaleFactor.get() * observation.reprojectionError());
    } else {
      xyStdDev *= (ambiguityScaleFactor.get() * (observation.poseAmbiguity() + 0.1));
    }

    // only trust the rotation component for multi-tag strategies; for single-tag, set the standard
    // deviation to infinity
    double thetaStdDev = observation.type() == PoseObservationType.MULTI_TAG
      ? thetaStdDevCoefficient.get()
        * Math.pow(observation.averageDistance(), 2.0)
        * (reprojectionErrorScaleFactor.get() * observation.reprojectionError())
        / observation.numberOfFiducials()
        * RobotConfiguration.getInstance().getCameraStdDevFactors()[index]
      : Double.POSITIVE_INFINITY;

    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  private void logCameraTransforms(int cameraIndex, PhotonFiducialResult observation) {
    // this is the pose of the robot when centered on the reef face that faces the driver station
    Pose3d cameraPose = observation.cameraPose();
    Transform3d robotToCameraTransform = cameraPose.minus(robotPosesForCalibration[cameraIndex]);

    Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/RobotToCameraTransform", robotToCameraTransform);
    Logger.recordOutput(VisionConstants.kSubsystemName + "/" + cameraIndex + "/RobotToCameraPose", robotPosesForCalibration[cameraIndex]);
  }
}