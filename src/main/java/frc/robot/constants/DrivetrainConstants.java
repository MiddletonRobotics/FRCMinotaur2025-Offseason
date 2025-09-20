package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class DrivetrainConstants {
    public static final String kSubsystemName = "Drivetrain";

    public static final double kDriveMaxSpeed = 4.43676260556;
    public static final double kDriveMaxAngularRate = Units.degreesToRadians(360 * 1.15);
    public static final double kDemoModeMaximumVelocity = 0.5;

    public static final double kHeadingControllerP = 3.5;
    public static final double kHeadingControllerI = 0;
    public static final double kHeadingControllerD = 0;

    public static final double kMaxSpeedMetersPerSecond = 4.75;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.85;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 6.0;
    public static final double kPYController = 2.0;
    public static final double kPThetaController = 4.0;

    public static final double kTranslationKa = 0.0;

    public static final boolean kEnableTeleportDetection = false;
    public static final double kTeleportDetectionThresholdMeters = 0.4;
    
    public static final double kTiltThresholdDegrees = 5.0;
    public static final double kUntiltingVelocityMetersPerSecond = 0.5;

    public enum SysIDCharacterizationMode {
        TRANSLATION_VOLTS,
        TRANSLATION_CURRENT,
        STEER_VOLTS,
        STEER_CURRENT,
        ROTATION_VOLTS
    }
}