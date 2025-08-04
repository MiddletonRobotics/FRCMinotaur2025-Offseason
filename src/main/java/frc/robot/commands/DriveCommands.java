package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.Drivetrain.DriveProfiles;
import frc.robot.utilities.constants.DrivetrainConstants;

public class DriveCommands {
    public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitide = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        linearMagnitide = Math.copySign(Math.pow(linearMagnitide, DrivetrainConstants.kDriveControlsExponent.get()), linearMagnitide);
        return new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitide, 0.0, new Rotation2d())).getTranslation();
    }

    public static Command teleopDrive(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, Boolean robotRelative) {
        return Commands.run(() -> {
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            double omega = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), 0.1, 0);
            omega = Math.copySign(Math.pow(omega, DrivetrainConstants.kDriveControlsExponent.get()), omega);

            ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * DrivetrainConstants.kMaxMeshedSpeed.in(MetersPerSecond), 
                linearVelocity.getY() * DrivetrainConstants.kMaxMeshedSpeed.in(MetersPerSecond), 
                omega * DrivetrainConstants.kTeleopRotationSpeed.get()
            );

            drivetrain.setDesiredChassisSpeeds(robotRelative ? speeds : ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, 
                DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? drivetrain.getPose().getRotation().plus(new Rotation2d(Math.PI)) : drivetrain.getPose().getRotation()
            ));
        }, drivetrain);
    }

    public static Command joystickDriveAtAngle(Drivetrain drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {

        ProfiledPIDController rotationController = new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.run(() -> {
            Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
            Rotation2d rotation = drivetrain.getPose().getRotation();
            double omega = rotationController.calculate(rotation.getRadians(), rotationSupplier.get().getRadians());

            ChassisSpeeds speeds = new ChassisSpeeds(
                    linearVelocity.getX() * DrivetrainConstants.kMaxMeshedSpeed.in(MetersPerSecond),
                    linearVelocity.getY() * DrivetrainConstants.kMaxMeshedSpeed.in(MetersPerSecond),
                    omega
            );

            boolean isFlipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
            drivetrain.setDesiredChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, isFlipped ? rotation.plus(new Rotation2d(Math.PI)) : rotation));
        },
        drivetrain).beforeStarting(() -> rotationController.reset(drivetrain.getPose().getRotation().getRadians()));
    }

    public static Command feedforwardCharacterization(Drivetrain drivetrain) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            Commands.runOnce(() -> {
                velocitySamples.clear();
                voltageSamples.clear();
                drivetrain.updateProfile(DriveProfiles.kDefault);
            }),

            Commands.run(() -> drivetrain.runCharacterization(0.0), drivetrain).withTimeout(2.0),
            Commands.runOnce(timer::restart),
            Commands.run(() -> {
                double voltage = timer.get() * 0.1;
                drivetrain.runCharacterization(voltage);
                velocitySamples.add(drivetrain.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
            }, drivetrain).finallyDo(() -> {
                int n = velocitySamples.size();

                double sumX = 0.0;
                double sumY = 0.0;
                double sumXY = 0.0;
                double sumX2 = 0.0;

                for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                }

                double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                NumberFormat formatter = new DecimalFormat("#0.00000");
                System.out.println("********** Drive FF Characterization Results **********");
                System.out.println("\tkS: " + formatter.format(kS));
                System.out.println("\tkV: " + formatter.format(kV));

                Logger.recordOutput("DrivetrainCharacterization/kS", formatter.format(kS));
                Logger.recordOutput("DrivetrainCharacterization/kV", formatter.format(kV));
            })
        );
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drivetrain drivetrain) {
        SlewRateLimiter limiter = new SlewRateLimiter(0.05);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            Commands.sequence(
                Commands.runOnce(() -> {
                    limiter.reset(0.0);
                    drivetrain.updateProfile(DriveProfiles.kDefault);
                }),

                Commands.run(() -> {
                    double speed = limiter.calculate(0.25);
                    drivetrain.setDesiredChassisSpeeds(new ChassisSpeeds(0.0, 0.0, speed));
                }, drivetrain)
            ),

            Commands.sequence(
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> {
                    state.positions = drivetrain.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drivetrain.getGyroRotation();
                    state.gyroDelta = 0.0;
                }),

                Commands.run(() -> {
                    var rotation = drivetrain.getGyroRotation();
                    state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                    state.lastAngle = rotation;
                }).finallyDo(() -> {
                    double[] positions = drivetrain.getWheelRadiusCharacterizationPositions();
                    double wheelDelta = 0.0;

                    for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                    }

                    double wheelRadius = (state.gyroDelta * DrivetrainConstants.kDriveBaseRadius.in(Meters)) / wheelDelta;

                    NumberFormat formatter = new DecimalFormat("#0.000000000000000000000000000");
                    System.out.println( "********** Wheel Radius Characterization Results **********");
                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                    System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                    System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, " + formatter.format(Units.metersToInches(wheelRadius)) + " inches");

                    Logger.recordOutput("DrivetrainCharacterization/WheelDelta", wheelDelta);
                    Logger.recordOutput("DrivetrainCharacterization/GyroDelta", state.gyroDelta);
                    Logger.recordOutput("DrivetrainCharacterization/WheelRadiusMeters", wheelRadius);
                    Logger.recordOutput("DrivetrainCharacterization/WheelRadiusInches", Units.metersToInches(wheelRadius));
                })
            )
        );
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}