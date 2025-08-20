package frc.minolib.wpilib;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * The PoseEstimate class represents an estimate of a robot's pose (position and orientation) at a
 * given point in time. It includes an identifier, the pose itself, and a flag indicating whether
 * vision data was used in the estimation.
 */
public class PoseEstimate {
    private final int id;
    private final Pose2d pose;
    private final boolean hasVision;

    public PoseEstimate() {
        this(0, Pose2d.kZero, false);
    }

    public PoseEstimate(final int id, final Pose2d pose, final boolean hasVision) {
        this.id = id;
        this.pose = pose;
        this.hasVision = hasVision;
    }

    public int getID() {
        return id;
    }

    public Pose2d getPose() {
        return pose;
    }

    public boolean hasVision() {
        return hasVision;
    }

    @Override
    public String toString() {
        return String.format("PoseEstimate(%s, %s, %s)", id, pose, hasVision);
    }

    // Struct for serialization.
    public static final PoseEstimateStruct struct = new PoseEstimateStruct();
}