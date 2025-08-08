package frc.robot.utilities.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.utilities.ModuleLimits;
import lombok.Builder;

public class DrivetrainConstants {
    public static final Distance kTrackWidthX = Inches.of(22.75);
    public static final Distance kTrackWidthY = Inches.of(22.75);
    public static final Distance kDriveBaseRadius = Meters.of(Math.hypot(kTrackWidthX.in(Meters) / 2.0, kTrackWidthY.in(Meters) / 2.0));

    public static final Current kDriveCurrentLimit = Amps.of(80.0);
    public static final Current kSteerCurrentLimit = Amps.of(40.0);

    public static final Distance kWheelRadius = Inches.of(1.917);
    public static final double kOdometryFrequency = new CANBus().isNetworkFD() ? 250.0 : 50.0; // Hz

    public static final SwerveModuleGearing kDrivetrainGearing = SwerveModuleGearing.MK4N_L2;
    public static final double kDriveGearRatio = kDrivetrainGearing.getDriveReduction();
    public static final double kTurnGearRatio = kDrivetrainGearing.getSteerReduction();

    public static final double kDriveSimGearRatio = kDriveGearRatio;
    public static final double kDriveSimMOI = 0.025;
    public static final double kTurnSimGearRatio = kTurnGearRatio;
    public static final double kTurnSimMOI = 0.004;

    public static final double kMaxLinearVelocity = 4.69;
    public static final double kMaxLinearAcceleration = 22.0;
    public static final double kMaxAngularVelocity = 4.69 / kDriveBaseRadius.baseUnitMagnitude();

    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthX.in(Meters) / 2.0, kTrackWidthY.in(Meters) / 2.0),
        new Translation2d(kTrackWidthX.in(Meters) / 2.0, -kTrackWidthY.in(Meters) / 2.0),
        new Translation2d(-kTrackWidthX.in(Meters) / 2.0, kTrackWidthY.in(Meters) / 2.0),
        new Translation2d(-kTrackWidthX.in(Meters) / 2.0, -kTrackWidthY.in(Meters) / 2.0)
    };

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
    public static final ModuleLimits kModuleLimitsFree = new ModuleLimits(kMaxAngularVelocity, kMaxLinearAcceleration, Units.degreesToRadians(1080.0));

    public static final ModuleConfiguration[] kCompetitionModuleConfiguration = {
        ModuleConfiguration.builder()
            .driveMotorID(2)
            .steerMotorID(3)
            .swerveEncoderID(10)
            .swerveEncoderOffset(Rotation2d.fromDegrees(0.0))
            .steerInverted(false)
            .swerveEncoderInverted(false)
            .build(),

        ModuleConfiguration.builder()
            .driveMotorID(4)
            .steerMotorID(5)
            .swerveEncoderID(11)
            .swerveEncoderOffset(Rotation2d.fromDegrees(0.0))
            .steerInverted(false)
            .swerveEncoderInverted(false)
            .build(),

        ModuleConfiguration.builder()
            .driveMotorID(6)
            .steerMotorID(7)
            .swerveEncoderID(12)
            .swerveEncoderOffset(Rotation2d.fromDegrees(0.0))
            .steerInverted(false)
            .swerveEncoderInverted(false)
            .build(),

        ModuleConfiguration.builder()
            .driveMotorID(8)
            .steerMotorID(9)
            .swerveEncoderID(13)
            .swerveEncoderOffset(Rotation2d.fromDegrees(0.0))
            .steerInverted(false)
            .swerveEncoderInverted(false)
            .build()
    };

    public static final int kPigeonID = 14;

    @Builder
    public record ModuleConfiguration(int driveMotorID, int steerMotorID, int swerveEncoderID, Rotation2d swerveEncoderOffset, boolean steerInverted, boolean swerveEncoderInverted) {}
}
