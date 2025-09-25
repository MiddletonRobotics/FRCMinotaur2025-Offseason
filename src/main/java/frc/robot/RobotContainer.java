// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
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
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private AprilTagFieldLayout layout;
  private RobotConfiguration robotConfiguration;
  private DrivetrainSubsystem drivetrain;
  private TeleopSwerve driveCommand;
  private VisionSubsystem vision;

  private CommandXboxController driverController;

  public RobotContainer() {
    robotConfiguration = new DefaultRobotConfiguration();

    configureControllers();
    configureSubsystems();
    configureBindings();
  }

  private void configureSubsystems() {
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    }

    drivetrain = new DrivetrainSubsystem(new DrivetrainIOCTRE());
    vision = new VisionSubsystem(new VisionIO[] {
      new VisionIOPhotonVision(VisionConstants.frontLeftCameraConfiguration, layout),
      new VisionIOPhotonVision(VisionConstants.frontRightCameraConfiguration, layout)
    });

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