// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.minolib.RobotConfiguration;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.DefaultRobotConfiguration;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.drivetrain.DrivetrainIOCTRE;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSimulation;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private AprilTagFieldLayout layout;
  private RobotConfiguration robotConfiguration;
  private DrivetrainSubsystem drivetrain;
  private TeleopSwerve driveCommand;
  private VisionSubsystem vision;

  private CommandXboxController driverController;

  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

  private static final String LAYOUT_FILE_MISSING = "Could not find the specified AprilTags layout file";
  private Alert layoutFileMissingAlert = new Alert(LAYOUT_FILE_MISSING, AlertType.kError);

  private Alert tuningAlert = new Alert("Tuning mode enabled", AlertType.kInfo);

  private DrivetrainSubsystem buildDrivetrain() {
    return new DrivetrainSubsystem(new DrivetrainIOCTRE());
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrain;
  }

  private VisionSubsystem buildVision() {
    if (RobotBase.isSimulation()) {
        VisionIO[] visionIOs = new VisionIO[VisionConstants.cameraConfigurations.length];
        AprilTagFieldLayout layout;

        try {
          layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
        } catch (IOException e) {
          layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
          layoutFileMissingAlert.setText(LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
          layoutFileMissingAlert.set(true);
        }

        for (int i = 0; i < visionIOs.length; i++) {
          visionIOs[i] = new VisionIOSimulation(VisionConstants.cameraConfigurations[i], layout, drivetrain::getPose);
        }

        return new VisionSubsystem(visionIOs);
      } else {
        VisionIO[] visionIOs = new VisionIO[VisionConstants.cameraConfigurations.length];
        AprilTagFieldLayout layout;

        try {
          layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
        } catch (IOException e) {
          layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
          layoutFileMissingAlert.setText(LAYOUT_FILE_MISSING + ": " + VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
          layoutFileMissingAlert.set(true);
        }

        for (int i = 0; i < visionIOs.length; i++) {
          visionIOs[i] = new VisionIOPhotonVision(VisionConstants.cameraConfigurations[i], layout);
        }

        return new VisionSubsystem(visionIOs);
      }
    }

    public VisionSubsystem getVisionSubsystem() {
      return vision;
    }

  public RobotContainer() {
    robotConfiguration = new DefaultRobotConfiguration();

    configureControllers();
    configureSubsystems();
    configureBindings();
  }

  private void configureSubsystems() {
    drivetrain = buildDrivetrain();
    vision = buildVision();

    driveCommand = new TeleopSwerve(drivetrain, driverController::getLeftY, driverController::getLeftX, driverController::getRightX);
  }

  private void configureControllers() {
    driverController = new CommandXboxController(0);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(driveCommand);

    driverController.a().onTrue(driveCommand.toggleFieldCentric());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}