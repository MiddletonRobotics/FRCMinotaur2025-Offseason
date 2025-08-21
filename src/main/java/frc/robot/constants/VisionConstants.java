package frc.robot.constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.minolib.vision.PipelineConfiguration;
import frc.minolib.wpilib.Fiducial;

public class VisionConstants {
    public static final class LeftElevator {
        public static final Transform3d robotToCameraT = new Transform3d(
            new Translation3d(Units.inchesToMeters(6.448335840566671), Units.inchesToMeters(12.433084524656367), Units.inchesToMeters(8.940524334006923)),
            new Rotation3d(Units.degreesToRadians(-0.6787726091457186), Units.degreesToRadians(-15.779523524036104), Units.degreesToRadians(-15.791622361429742)));

        public static final PipelineConfiguration[] pipelineConfigs = new PipelineConfiguration[] {
            new PipelineConfiguration(
                Fiducial.Type.APRILTAG,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    916.965230021908,
                    0.0,
                    661.6928938560056,
                    0.0,
                    916.8276406094255,
                    435.5533504564346,
                    0.0,
                    0.0,
                    1.0
                ),
                MatBuilder.fill(
                    Nat.N8(),
                    Nat.N1(),
                    0.05009856981900392,
                    -0.07369910749297018,
                    -1.0660525417228317E-5,
                    1.3422933851637837E-4,
                    0.009667561013012865,
                    0.0,
                    0.0,
                    0.0
                )
            ),
        };
    }

    public static final class RightElevator {
        public static final Transform3d robotToCameraT = new Transform3d(
            new Translation3d(Units.inchesToMeters(6.532495676417899), Units.inchesToMeters(-12.67579593665854), Units.inchesToMeters(8.637531861540134)),
            new Rotation3d(Units.degreesToRadians(-0.5133124038290454),Units.degreesToRadians(-16.560452484503056), Units.degreesToRadians(15.538407387175491)));

        public static final PipelineConfiguration[] pipelineConfigs = new PipelineConfiguration[] {
            new PipelineConfiguration(
                Fiducial.Type.APRILTAG,
                1280,
                800,
                MatBuilder.fill(
                    Nat.N3(),
                    Nat.N3(),
                    903.7449893954214,
                    0.0,
                    654.6877054577706,
                    0.0,
                    903.9577323243168,
                    369.13545590011745,
                    0.0,
                    0.0,
                    1.0
                ),
                MatBuilder.fill(
                    Nat.N8(),
                    Nat.N1(),
                    0.05318440103944059,
                    -0.07489968261371575,
                    -5.477461690531807E-4,
                    -7.032312933604217E-4,
                    0.009722692505020142,
                    0.0,
                    0.0,
                    0.0
                )
            ),
        };
    }
}
