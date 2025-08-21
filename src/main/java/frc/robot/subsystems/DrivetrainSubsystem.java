package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.minolib.hardware.MinoPigeon2;
import frc.minolib.interfaces.MinoVisionCamera;
import frc.minolib.swerve.MinoSwerve;
import frc.minolib.swerve.MinoSwerveModule;
import frc.minolib.swerve.MinoSwerveModuleFactory;
import frc.minolib.vision.MinoVisionSimulation;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.utilities.Fiducials;

import java.util.ArrayList;

public class DrivetrainSubsystem extends MinoSwerve {
    public DrivetrainSubsystem(MinoPigeon2 imu, ArrayList<MinoVisionCamera> cameras, MinoVisionSimulation visionSim, Field2d fieldViz) {
        super(
            imu,
            cameras,
            DrivetrainConstants.maxDriveSpeed,
            DrivetrainConstants.maxModuleAcceleration,
            DrivetrainConstants.maxModuleSteeringRate,
            DrivetrainConstants.driveController,
            Fiducials.aprilTagFiducials,
            visionSim,
            fieldViz
        );
    }

    protected MinoSwerveModule[] createModules() {
        MinoSwerveModuleFactory swerveModuleFactory = new MinoSwerveModuleFactory(
            DrivetrainConstants.wheelCircumference,
            DrivetrainConstants.driveRatio,
            DrivetrainConstants.steeringRatio,
            DrivetrainConstants.steerDriveCouplingRatio,
            DrivetrainConstants.driveOpenLoopPIDConfig,
            DrivetrainConstants.driveClosedLoopPIDConfig,
            DrivetrainConstants.steeringPIDConfig
        );

        MinoSwerveModule[] modules = {
            swerveModuleFactory.createModule(
                DrivetrainConstants.FrontLeft.modulePosition,
                DrivetrainConstants.FrontLeft.driveMotorID,
                DrivetrainConstants.FrontLeft.steeringMotorID,
                DrivetrainConstants.FrontLeft.canCoderID,
                DrivetrainConstants.FrontLeft.absEncoderOffsetRad
            ),
            swerveModuleFactory.createModule(
                DrivetrainConstants.RearLeft.modulePosition,
                DrivetrainConstants.RearLeft.driveMotorID,
                DrivetrainConstants.RearLeft.steeringMotorID,
                DrivetrainConstants.RearLeft.canCoderID,
                DrivetrainConstants.RearLeft.absEncoderOffsetRad
            ),
            swerveModuleFactory.createModule(
                DrivetrainConstants.RearRight.modulePosition,
                DrivetrainConstants.RearRight.driveMotorID,
                DrivetrainConstants.RearRight.steeringMotorID,
                DrivetrainConstants.RearRight.canCoderID,
                DrivetrainConstants.RearRight.absEncoderOffsetRad
            ),
            swerveModuleFactory.createModule(
                DrivetrainConstants.FrontRight.modulePosition,
                DrivetrainConstants.FrontRight.driveMotorID,
                DrivetrainConstants.FrontRight.steeringMotorID,
                DrivetrainConstants.FrontRight.canCoderID,
                DrivetrainConstants.FrontRight.absEncoderOffsetRad
            ),
        };
        
        return modules;
    }
}