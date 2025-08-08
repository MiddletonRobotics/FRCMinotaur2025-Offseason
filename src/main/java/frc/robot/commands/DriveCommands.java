package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.utilities.constants.DrivetrainConstants;

public class DriveCommands {
    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    
        linearMagnitude = linearMagnitude * linearMagnitude;
    
        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection).transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d())).getTranslation();
    }

    public static double getOmegaFromJoysticks(double driverOmega) {
        double omega = MathUtil.applyDeadband(driverOmega, 0.1);
        return omega * omega * Math.signum(omega);
    }

    public static ChassisSpeeds getSpeedsFromJoysticks(double driverX, double driverY, double driverOmega) {
        // Get linear velocity
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(driverX, driverY).times(DrivetrainConstants.kMaxLinearVelocity);
        double omega = getOmegaFromJoysticks(driverOmega);

        return new ChassisSpeeds(linearVelocity.getX(), linearVelocity.getY(), omega * DrivetrainConstants.kMaxAngularVelocity);
    }

    public static Command teleopDrive(Drivetrain drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier, BooleanSupplier robotRelative) {
        return Commands.run(() -> {
            ChassisSpeeds speeds = getSpeedsFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble(), omegaSupplier.getAsDouble());
            drive.runVelocity(robotRelative.getAsBoolean()
                ? speeds
                : ChassisSpeeds.fromFieldRelativeSpeeds(speeds, DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red
                    ? drive.getPose().getRotation().plus(Rotation2d.kPi)
                    : drive.getPose().getRotation()
                )
            );
        }, drive);
    }
}