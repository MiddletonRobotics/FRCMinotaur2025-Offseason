package frc.robot.constants;

import java.io.File;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static double[] cameraStdDevFactors = new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
    };

    public static final double maxAmbiguity = 0.3;
    public static final double maxErrorZ = 0.75;

    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
