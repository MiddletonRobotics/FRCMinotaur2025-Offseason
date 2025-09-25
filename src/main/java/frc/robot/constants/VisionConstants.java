package frc.robot.constants;

import java.io.File;
import java.nio.file.Path;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.minolib.vision.CameraConfiguration;
import frc.minolib.vision.PipelineConfiguration;
import frc.minolib.vision.CameraConfiguration.CameraLocation;
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

    public static final String kSubsystemName = "VisionSubsystem";

    public static final CameraConfiguration frontLeftCameraConfiguration = new CameraConfiguration()
        .withCameraName("le")
        .withHeightOffset(0)
        .withLengthOffset(0)
        .withWidthOffset(0)
        .withMountingPitch(0)
        .withMountingRoll(0)
        .withMountingYaw(0)
        .withCameraLocation(CameraLocation.FRONT_LEFT);

    public static final CameraConfiguration frontRightCameraConfiguration = new CameraConfiguration("re")
        .withCameraName("re")
        .withHeightOffset(0)
        .withLengthOffset(0)
        .withWidthOffset(0)
        .withMountingPitch(0)
        .withMountingRoll(0)
        .withMountingYaw(0)
        .withCameraLocation(CameraLocation.FRONT_LEFT);

    public static final boolean kDetailedLoggnig = false;
    public static final boolean kCalibrateCameraTranformation = false;
    
    public static final double kBestPoseThreshold = 0.5;
    public static final double kTagLogThreshold = 0.1;
    public static final double kPoseStalenessThreshold = 0.1;
    
    public static final double kAmbiguityThreshold = 0.5;
    public static final double kRotationThreshold = 10.0;
    public static final double kReprojectionErrorThreshold = 5.0;
    
    public static final double maxZErrorMeters = 0.25;
    public static final double kFieldBorderMarginError = 0.5;
    
    public static final double kAmbiguityScaleFactor = 5.0;
    public static final double kReprojectionErrorScaleFactor = 3.33;
    
    // the coefficient from which the standard deviation for the x and y components is initiated
    public static final double linearStandardDeviation = 0.08;
    public static final double thetaStandardDeviation = 0.1;
    
    // the average error in pixels for the simulated camera
    public static final double SIM_AVERAGE_ERROR_PIXELS = 0.1;
    public static final double SIM_ERROR_STD_DEV_PIXELS = 0.05;

    public static final Path APRILTAG_FIELD_LAYOUT_PATH = new File(Filesystem.getDeployDirectory(), "2025-reefscape.json").toPath();
    public static final Path OFFICIAL_APRILTAG_FIELD_LAYOUT_PATH = new File(Filesystem.getDeployDirectory(), "2025-reefscape.json").toPath();
    public static final int MAX_NUMBER_TAGS = 30;

}
