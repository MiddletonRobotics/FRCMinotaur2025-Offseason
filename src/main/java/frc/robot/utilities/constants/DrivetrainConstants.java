package frc.robot.utilities.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.utilities.LoggedTunableNumber;
import frc.robot.utilities.SwerveModuleGearing;
import frc.robot.utilities.SwerveSetpointGenerator.ModuleLimits;

public class DrivetrainConstants {
    public static final LoggedTunableNumber kDriveToPointP = new LoggedTunableNumber("DriveToPoint P", 3.2);
    public static final LoggedTunableNumber kDriveToPointI = new LoggedTunableNumber("DriveToPoint I", 0.0);
    public static final LoggedTunableNumber kDriveToPointD = new LoggedTunableNumber("DriveToPoint D", 0.18);

    public static final LoggedTunableNumber kDriveToPointAutoP = new LoggedTunableNumber("DriveToPoint Auto P", 3.0);
    public static final LoggedTunableNumber kDriveToPointAutoI = new LoggedTunableNumber("DriveToPoint Auto I", 0.0);
    public static final LoggedTunableNumber kDriveToPointAutoD = new LoggedTunableNumber("DriveToPoint Auto D", 0.12);

    public static final LoggedTunableNumber kDriveToPointHeadingP = new LoggedTunableNumber("DriveToPoint Heading P", 4.0);
    public static final LoggedTunableNumber kDriveToPointHeadingI = new LoggedTunableNumber("DriveToPoint Heading I", 0.0);
    public static final LoggedTunableNumber kDriveToPointHeadingD = new LoggedTunableNumber("DriveToPoint Heading D", 0.05);

    public static final LoggedTunableNumber kMeshedXYP = new LoggedTunableNumber("Drive Meshed XY P", 3.0);
    public static final LoggedTunableNumber kMeshedXYD = new LoggedTunableNumber("Drive Meshed XY D", 0.12);
    public static final LoggedTunableNumber kMeshedThetaP = new LoggedTunableNumber("Drive Meshed Theta P", 3.0);
    public static final LoggedTunableNumber kMeshedThetaD = new LoggedTunableNumber("Drive Meshed Theta D", 0.0);
    public static final LoggedTunableNumber kDebounceAmount = new LoggedTunableNumber("Meshed Drive Debounce", 0.1);
    public static final LoggedTunableNumber kMeshDrivePriority = new LoggedTunableNumber("Meshed Drive Priority", 0.3);

    public static final LoggedTunableNumber kTeleopRotationSpeed = new LoggedTunableNumber("Teleop Rotation Speed", 10.0);
    public static final LoggedTunableNumber kDriveControlsExponent = new LoggedTunableNumber("Drive Control Mode", 2.0);
    public static final double kFreefallAccelerationThreshold = 9.0;

    public static final LinearVelocity kMaxMeshedSpeed = MetersPerSecond.of(4.5);
    public static final LinearVelocity kMaxDriveToPointSpeed = MetersPerSecond.of(3.6);
    public static final LinearAcceleration kMaxLinearAcceleration = MetersPerSecondPerSecond.of(3.0);
    public static final Distance kTrackWidthX = Inches.of(22.75);
    public static final Distance kTrackWidthY = Inches.of(22.75);
    public static final Distance kDriveBaseRadius = Inches.of(Math.hypot(kTrackWidthX.baseUnitMagnitude() / 2.0, kTrackWidthY.baseUnitMagnitude() / 2.0));
    public static final AngularVelocity kMaxAngularSpeed = RadiansPerSecond.of(kMaxMeshedSpeed.baseUnitMagnitude() / kDriveBaseRadius.in(Meters));
    public static final AngularAcceleration kMaxAngularAcceleration = RadiansPerSecondPerSecond.of(kMaxLinearAcceleration.baseUnitMagnitude() / kDriveBaseRadius.in(Meters));
    public static final LinearVelocity kVisionSpeedConstantK = MetersPerSecond.of(0.5);

    public static final Current kDriveDefaultSupplyCurrentLimit = Amps.of(70.0);
    public static final Current kDriveDefaultStatorCurrentLimit = Amps.of(120.0);
    public static final Current kSteerDefaultSupplyCurrentLimit = Amps.of(40.0);
    public static final Current kSteerDefaultStatorCurrentLimit = Amps.of(80.0);

    public static final SwerveModuleGearing kDrivetrainGearing = SwerveModuleGearing.MK4N_L2;
    public static final double kDriveGearRatio = kDrivetrainGearing.getDriveReduction();
    public static final double kTurnGearRatio = kDrivetrainGearing.getSteerReduction();

    public static final Distance kWheelRadius = Inches.of(1.917);
    public static final double kOdometryFrequency = 250.0;

    public static final double kDriveSimGearRatio = kDriveGearRatio;
    public static final double kDriveSimMOI = 0.025;
    public static final double kTurnSimGearRatio = kTurnGearRatio;
    public static final double kTurnSimMOI = 0.004;

    public static final boolean kRealReversed = false;
    public static final boolean kSimReversed = false;

    public interface Ports {
        public static final int kFrontLeftDriveMotorID = 2;
        public static final int kFrontLeftSteerMotorID = 3;
        public static final int kFrontLeftSwerveEncoderID = 10;

        public static final int kFrontRightDriveMotorID = 4;
        public static final int kFrontRightSteerMotorID = 5;
        public static final int kFrontRightSwerveEncoderID = 11;

        public static final int kBackLeftDriveMotorID = 6;
        public static final int kBackLeftSteerMotorID = 7;
        public static final int kBackLeftSwerveEncoderID = 12;

        public static final int kBackRightDriveMotorID = 8;
        public static final int kBackRightSteerMotorID = 9;
        public static final int kBackRightSwerveEncoderID = 13;

        public static final int kSwerveGyroID = 14;

        public static final String kDrivetrainCanivoreName = "Drivetrain";
    }

    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthX.in(Meters) / 2.0, kTrackWidthY.in(Meters) / 2.0),
        new Translation2d(kTrackWidthX.in(Meters) / 2.0, -kTrackWidthY.in(Meters) / 2.0),
        new Translation2d(-kTrackWidthX.in(Meters) / 2.0, kTrackWidthY.in(Meters) / 2.0),
        new Translation2d(-kTrackWidthX.in(Meters) / 2.0, -kTrackWidthY.in(Meters) / 2.0)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
    public static final ModuleLimits kModuleLimitsFree = new ModuleLimits(kMaxMeshedSpeed.baseUnitMagnitude(), kMaxAngularSpeed.baseUnitMagnitude(), Units.degreesToRadians(1080.0));
}
