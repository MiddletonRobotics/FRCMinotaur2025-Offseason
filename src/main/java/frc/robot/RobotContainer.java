// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.minolib.hardware.MinoPigeon2;
import frc.minolib.interfaces.MinoVisionCamera;
import frc.minolib.vision.MinoVisionSimulation;
import frc.minolib.vision.PhotonVisionCamera;
import frc.robot.command_factories.DriveFactory;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utilities.Fiducials;

public class RobotContainer {
  private DrivetrainSubsystem drivetrainSubsystem;
  public final XboxController driverXbox = new XboxController(0);

  private final ArrayList<MinoVisionCamera> cameras = new ArrayList<>(Arrays.asList(
    new PhotonVisionCamera("le", VisionConstants.LeftElevator.robotToCameraT, VisionConstants.LeftElevator.pipelineConfigs),
    new PhotonVisionCamera("re", VisionConstants.RightElevator.robotToCameraT, VisionConstants.RightElevator.pipelineConfigs)
  ));

  private final MinoPigeon2 imu = new MinoPigeon2(DrivetrainConstants.IMU.pigeonID, MinoPigeon2.makeDefaultConfig());
  private final ArrayList<MinoVisionCamera> localizationCameras = new ArrayList<>(Arrays.asList(cameras.get(0), cameras.get(1)));
  private final MinoVisionSimulation visionSimulation = new MinoVisionSimulation(cameras, Fiducials.aprilTagFiducials);
  private final Field2d fieldViz = visionSimulation.getSimField();

  public RobotContainer() {
    configureSubsystems();
    configureControllers();
    configureBindings();
  }

  private void configureSubsystems() {
    drivetrainSubsystem = new DrivetrainSubsystem(
        imu,
        localizationCameras,
        visionSimulation,
        fieldViz
    );

    drivetrainSubsystem.setDefaultCommand(new DriveFactory(drivetrainSubsystem, new XboxController(0)));
  }

  private void configureControllers() {

  }

  private void configureBindings() {
    drivetrainSubsystem.setDefaultCommand(new DriveFactory(drivetrainSubsystem, driverXbox));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
