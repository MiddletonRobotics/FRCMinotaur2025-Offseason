package frc.robot.command_factories;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DrivetrainFactory {
    private static SwerveRequest.RobotCentric driveRobotCentricRequest = new SwerveRequest.RobotCentric();
    private static SwerveRequest.FieldCentric driveFieldCentricRequest = new SwerveRequest.FieldCentric();

    public static Command handleTeleopDrive(DrivetrainSubsystem drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, boolean isFieldRelative) {
        return Commands.run(() -> {
            ChassisSpeeds chassisSpeeds = calculateSpeedsBasedOnJoystickInputs(drivetrain, throttleSupplier, strafeSupplier, rotationSupplier);
            drivetrain.setTargetChassisSpeeds(chassisSpeeds);
            
            if(isFieldRelative) {
                drivetrain.setControl(driveFieldCentricRequest
                    .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                    .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                    .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond)
                );
            } else {
                drivetrain.setControl(driveRobotCentricRequest
                    .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                    .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                    .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond)
                );
            }
        }, drivetrain);
    }

    private static ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(DrivetrainSubsystem drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = MathUtil.applyDeadband(throttleSupplier.getAsDouble(), GlobalConstants.kControllerDeadband);
        double yMagnitude = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), GlobalConstants.kControllerDeadband);
        double angularMagnitude = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), GlobalConstants.kControllerDeadband);
        
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity = (GlobalConstants.isBlueAlliance() ? -xMagnitude * DrivetrainConstants.kDriveMaxSpeed : xMagnitude * DrivetrainConstants.kDriveMaxSpeed) * drivetrain.getTeleopVelocityCoefficent().getTranslationalCoefficient();
        double yVelocity = (GlobalConstants.isBlueAlliance() ? -yMagnitude * DrivetrainConstants.kDriveMaxSpeed : yMagnitude * DrivetrainConstants.kDriveMaxSpeed) * drivetrain.getTeleopVelocityCoefficent().getTranslationalCoefficient();
        double angularVelocity = angularMagnitude * DrivetrainConstants.kDriveMaxAngularRate * drivetrain.getTeleopVelocityCoefficent().getRotationalCoefficient();

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond * -0.03); //TODO: Some skew comp to be put in constants

        return ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xVelocity, yVelocity, -angularVelocity), drivetrain.getPose().getRotation()),
            drivetrain.getPose().getRotation().plus(skewCompensationFactor)
        );
    }
}
