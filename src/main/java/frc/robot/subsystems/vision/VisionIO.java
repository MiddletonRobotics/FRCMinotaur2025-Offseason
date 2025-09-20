package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import frc.minolib.vision.CameraConfiguration;
import frc.minolib.vision.PhotonFiducialResult;
import frc.minolib.vision.CameraConfiguration.CameraLocation;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
        public boolean isCameraConnected = false;
        public PhotonFiducialResult[] visionPacket = new PhotonFiducialResult[0];
    }

    public default void updateInputs(VisionIOInputs inputs) {}

    public default CameraLocation getCameraLocation() {
        return CameraLocation.NONE;
    }

    public default String getCameraName() {
        return "";
    }

    public enum PoseObservationType {
        SINGLE_TAG,
        MULTI_TAG
    }
}
