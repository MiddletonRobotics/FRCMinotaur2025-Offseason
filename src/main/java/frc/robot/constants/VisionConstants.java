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
    public static final String kSubsystemName = "VisionSubsystem";

    public static final CameraConfiguration frontLeftCameraConfiguration = new CameraConfiguration()
        .withCameraName("le")
        .withHeightOffset(Units.inchesToMeters(8))
        .withLengthOffset(Units.inchesToMeters(11.125))
        .withWidthOffset(Units.inchesToMeters(7.446))
        .withMountingPitch(0)
        .withMountingRoll(0)
        .withMountingYaw(0)
        .withCameraLocation(CameraLocation.FRONT_LEFT);
            
    public static final CameraConfiguration frontRightCameraConfiguration = new CameraConfiguration()
        .withCameraName("re")
        .withHeightOffset(Units.inchesToMeters(8))
        .withLengthOffset(Units.inchesToMeters(11.125))
        .withWidthOffset(Units.inchesToMeters(-7.446))
        .withMountingPitch(0)
        .withMountingRoll(0)
        .withMountingYaw(0)
        .withCameraLocation(CameraLocation.FRONT_RIGHT);

    public static final CameraConfiguration[] cameraConfigurations = new CameraConfiguration[] {
        frontLeftCameraConfiguration,
        frontRightCameraConfiguration
    };

    public static final boolean kDetailedLoggnig = true;
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
