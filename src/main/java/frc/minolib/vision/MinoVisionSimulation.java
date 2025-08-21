package frc.minolib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.minolib.interfaces.MinoVisionCamera;
import frc.minolib.wpilib.Fiducial;
import frc.robot.Robot;
import java.util.ArrayList;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

public class MinoVisionSimulation {
    private final VisionSystemSim visionSimulator = new VisionSystemSim("main");

    public MinoVisionSimulation(final ArrayList<MinoVisionCamera> cameras, final Fiducial[] aprilTags) {
        if (Robot.isReal()) {
            return;
        }

        for (var camera : cameras) {
            switch (camera.getPipelineConfiguration().fiducialType) {
                case APRILTAG -> {
                    for (var tag : aprilTags) {
                        visionSimulator.addVisionTargets("apriltag", new VisionTargetSim(tag.getPose(), TargetModel.kAprilTag36h11, tag.id()));
                    }
                    break;
                }

                default -> {
                    break;
                }
            }

            visionSimulator.addCamera(camera.getCameraSim(), camera.getTransform());
        }
    }

    public void resetSimPose(final Pose2d pose) {
        visionSimulator.resetRobotPose(pose);
    }

    public void updatePose(final Pose2d pose) {
        visionSimulator.update(pose);
    }

    public Field2d getSimField() {
        return visionSimulator.getDebugField();
    }
}