package frc.minolib.vision;

import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;

/** A data class for a pipeline packet. */
public class PipelineVisionPacket {
    private final boolean hasTargets;
    private final PhotonTrackedTarget bestTarget;
    private final List<PhotonTrackedTarget> targets;
    private final double captureTimestamp;

    public PipelineVisionPacket(final boolean hasTargets, final PhotonTrackedTarget bestTarget, final List<PhotonTrackedTarget> targets, final double captureTimestamp) {
        this.hasTargets = hasTargets;
        this.bestTarget = bestTarget;
        this.targets = targets;
        this.captureTimestamp = captureTimestamp;
    }

    /**
     * If the vision packet has valid targets.
     *
     * @return if targets are found.
     */
    public boolean hasTargets() {
        return hasTargets;
    }

    /**
     * Gets best target.
     *
     * @return the best target.
     */
    public PhotonTrackedTarget getBestTarget() {
        return bestTarget;
    }

    /**
     * Gets targets.
     *
     * @return the targets.
     */
    public List<PhotonTrackedTarget> getTargets() {
        return targets;
    }

    /**
     * Gets the capture timestamp in seconds. -1 if the result has no timestamp set.
     *
     * @return the timestamp in seconds.
     */
    public double getCaptureTimestamp() {
        return captureTimestamp;
    }
}