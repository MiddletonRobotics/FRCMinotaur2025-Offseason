package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Timer;
import frc.minolib.vision.CameraConfiguration;
import frc.minolib.vision.PhotonFiducialResult;
import frc.minolib.vision.CameraConfiguration.CameraLocation;

public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;
    protected PhotonPoseEstimator poseEstimator;
    private CameraLocation cameraLocation;
    private final List<PhotonFiducialResult> visionResults = new ArrayList<>();

    public VisionIOPhotonVision(CameraConfiguration cameraConfiguration, AprilTagFieldLayout fieldLayout) {
        camera = new PhotonCamera(cameraConfiguration.getCameraName());
        cameraLocation = cameraConfiguration.getLocation();
        poseEstimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS, cameraConfiguration.getTransformOffset());

        this.camera.getAllUnreadResults();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.isCameraConnected = camera.isConnected();
        visionResults.clear();

        for(PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

            estimatedPose.ifPresent(estimate -> {
                long tagsSeenBitMap = 0;
                double averageAmbiguity = 0.0;
                double averageTagDistance = 0.0;

                for(int i = 0; i < estimate.targetsUsed.size(); i++) {
                    tagsSeenBitMap |= 1L << estimate.targetsUsed.get(i).getFiducialId();
                    averageAmbiguity += estimate.targetsUsed.get(i).getPoseAmbiguity();
                    averageTagDistance += estimate.targetsUsed.get(i).getBestCameraToTarget().getTranslation().getNorm();
                }

                averageAmbiguity /= estimate.targetsUsed.size();
                averageTagDistance /= estimate.targetsUsed.size();

                visionResults.add(new PhotonFiducialResult(
                    result.getTargets().size(),
                    averageAmbiguity, 
                    estimate.estimatedPose, 
                    averageTagDistance,
                    result.multitagResult.isPresent() ? result.multitagResult.get().estimatedPose.bestReprojErr : 0.0,
                    tagsSeenBitMap,
                    result.getTimestampSeconds(),
                    Timer.getFPGATimestamp() - result.getTimestampSeconds(),
                    estimate.strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ? PoseObservationType.MULTI_TAG : PoseObservationType.SINGLE_TAG
                ));
            });
        }

        inputs.visionPacket = visionResults.toArray(new PhotonFiducialResult[0]);
    }

    @Override
    public CameraLocation getCameraLocation() {
        return cameraLocation;
    }

    @Override
    public String getCameraName() {
        return camera.getName();
    }
}
