package frc.robot.command_factories;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.ReefConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DrivetrainFactory {
    private static SwerveRequest.RobotCentric driveRobotCentricRequest = new SwerveRequest.RobotCentric();
    private static SwerveRequest.FieldCentric driveFieldCentricRequest = new SwerveRequest.FieldCentric();
    private static SwerveRequest.FieldCentricFacingAngle driveFieldFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.Velocity);

    private static final PIDController autonomousDriveToPointController = new PIDController(3.0, 0, 0.1);
    private static final PIDController teleopDriveToPointController = new PIDController(3.6, 0, 0.1);

    public static Command handleTeleopDrive(DrivetrainSubsystem drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, boolean isFieldRelative) {
        return Commands.run(() -> {
            ChassisSpeeds chassisSpeeds = calculateSpeedsBasedOnJoystickInputs(drivetrain, throttleSupplier, strafeSupplier, rotationSupplier);
            drivetrain.setTargetChassisSpeeds(chassisSpeeds); // Used only for logging
            
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

    public static Command driveToPoint(DrivetrainSubsystem drivetrain, Pose2d desiredPoseForDriveToPoint, double constraintedMaximumLinearVelocity, double constraintedMaximumAngularVelocity) {
        return Commands.run(() -> {
            Translation2d translationToDesiredPoint = desiredPoseForDriveToPoint.getTranslation().minus(drivetrain.getPose().getTranslation());
            double linearDistance = translationToDesiredPoint.getNorm();
            double frictionConstant = 0.0;

            if (linearDistance >= Units.inchesToMeters(0.5)) {
                frictionConstant = 0.02 * DrivetrainConstants.kDriveMaxSpeed;
            }

            Rotation2d directionOfTravel = translationToDesiredPoint.getAngle();
            double velocityOutput = 0.0;

            if (DriverStation.isAutonomous()) {
                velocityOutput = Math.min(
                    Math.abs(autonomousDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                    constraintedMaximumLinearVelocity
                );
            } else {
                velocityOutput = Math.min(
                    Math.abs(teleopDriveToPointController.calculate(linearDistance, 0)) + frictionConstant,
                    constraintedMaximumLinearVelocity
                );
            }

            double xComponent = velocityOutput * directionOfTravel.getCos();
            double yComponent = velocityOutput * directionOfTravel.getSin();

            Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/DriveToPoint/xVelocitySetpoint", xComponent);
            Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/DriveToPoint/yVelocitySetpoint", yComponent);
            Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/DriveToPoint/velocityOutput", velocityOutput);
            Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/DriveToPoint/linearDistance", linearDistance);
            Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/DriveToPoint/directionOfTravel", directionOfTravel);
            Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/DriveToPoint/desiredPoint", desiredPoseForDriveToPoint);

            if (Double.isNaN(constraintedMaximumAngularVelocity)) {
                drivetrain.setControl(driveFieldFacingAngle
                    .withVelocityX(xComponent)
                    .withVelocityY(yComponent)
                    .withTargetDirection(desiredPoseForDriveToPoint.getRotation())
                );
            } else {
                drivetrain.setControl(driveFieldFacingAngle
                    .withVelocityX(xComponent)
                    .withVelocityY(yComponent)
                    .withTargetDirection(desiredPoseForDriveToPoint.getRotation())
                    .withMaxAbsRotationalRate(constraintedMaximumAngularVelocity)
                );
            }
        }, drivetrain).until(() -> drivetrain.isAtDriveToPointSetpoint(desiredPoseForDriveToPoint));
    }


    private static ChassisSpeeds calculateSpeedsBasedOnJoystickInputs(DrivetrainSubsystem drivetrain, DoubleSupplier throttleSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        if (DriverStation.getAlliance().isEmpty()) {
            return new ChassisSpeeds(0, 0, 0);
        }

        double xMagnitude = MathUtil.applyDeadband(throttleSupplier.getAsDouble(), GlobalConstants.kControllerDeadband);
        double yMagnitude = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), GlobalConstants.kControllerDeadband);
        double angularMagnitude = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), GlobalConstants.kControllerDeadband);
        
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        double xVelocity = (GlobalConstants.isBlueAlliance() ? xMagnitude * DrivetrainConstants.kDriveMaxSpeed : -xMagnitude * DrivetrainConstants.kDriveMaxSpeed) * drivetrain.getTeleopVelocityCoefficent().getTranslationalCoefficient();
        double yVelocity = (GlobalConstants.isBlueAlliance() ? yMagnitude * DrivetrainConstants.kDriveMaxSpeed : -yMagnitude * DrivetrainConstants.kDriveMaxSpeed) * drivetrain.getTeleopVelocityCoefficent().getTranslationalCoefficient();
        double angularVelocity = angularMagnitude * DrivetrainConstants.kDriveMaxAngularRate * drivetrain.getTeleopVelocityCoefficent().getRotationalCoefficient();

        Rotation2d skewCompensationFactor = Rotation2d.fromRadians(drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond * -0.03); //TODO: Some skew comp to be put in constants

        return ChassisSpeeds.fromRobotRelativeSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xVelocity, yVelocity, angularVelocity), drivetrain.getPose().getRotation()),
            drivetrain.getPose().getRotation().plus(skewCompensationFactor)
        );
    }
}