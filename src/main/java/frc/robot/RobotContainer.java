// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.constants.VisionConstants;
import frc.robot.oi.Controlboard;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.DrivetrainIOCTRE;
import frc.robot.subsystems.drivetrain.DrivetrainIOSimulation;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.leds.LedIOCANdle;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSimulation;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private DrivetrainSubsystem drivetrain;
  private VisionSubsystem vision;
  private LedSubsystem ledSubsystem;

  private CommandXboxController driverController;
  private Controlboard controlboard;
  
  private final LoggedNetworkNumber endgameAlert1 = new LoggedNetworkNumber("/Tuning/Endgame Alert #1", 20.0);
  private final LoggedNetworkNumber endgameAlert2 = new LoggedNetworkNumber("/Tuning/Endgame Alert #2", 10.0);

  private DrivetrainSubsystem buildDrivetrain() {
    if(Robot.isSimulation()) {
      return new DrivetrainSubsystem(new DrivetrainIOSimulation(CompetitionTunerConstants.DrivetrainConstants, CompetitionTunerConstants.FrontLeft, CompetitionTunerConstants.FrontRight, CompetitionTunerConstants.BackLeft, CompetitionTunerConstants.BackRight));
    } else {
      return new DrivetrainSubsystem(new DrivetrainIOCTRE(CompetitionTunerConstants.DrivetrainConstants, CompetitionTunerConstants.FrontLeft, CompetitionTunerConstants.FrontRight, CompetitionTunerConstants.BackLeft, CompetitionTunerConstants.BackRight));
    }
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrain;
  }

  private VisionSubsystem buildVision() {
    if(Robot.isSimulation()) {
      return new VisionSubsystem(
        drivetrain, 
        new VisionIOSimulation(VisionConstants.frontLeftCameraConfiguration.getName(), VisionConstants.frontLeftCameraConfiguration.getTransformOffset(), drivetrain::getPose),
        new VisionIOSimulation(VisionConstants.frontRightCameraConfiguration.getName(), VisionConstants.frontRightCameraConfiguration.getTransformOffset(), drivetrain::getPose)
      );
    } else {
      return new VisionSubsystem(
        drivetrain,
        new VisionIOPhotonVision(VisionConstants.frontLeftCameraConfiguration.getName(), VisionConstants.frontLeftCameraConfiguration.getTransformOffset()),
        new VisionIOPhotonVision(VisionConstants.frontRightCameraConfiguration.getName(), VisionConstants.frontRightCameraConfiguration.getTransformOffset())
      );
    }
  }

  public VisionSubsystem getVisionSubsystem() {
    return vision;
  }

  public LedSubsystem buildLedSubsystem() {
    return new LedSubsystem(new LedIOCANdle());
  }

  public LedSubsystem getLedSubsystem() {
    return ledSubsystem;
  }

  public Controlboard buildControlboad() {
    return Controlboard.getInstance();
  }

  public Controlboard getControlboard() {
    return controlboard;
  }

  public RobotContainer() {
    configureControllers();
    configureSubsystems();
    configureBindings();
  }

  private void configureControllers() {
    controlboard = buildControlboad();
  }

  private void configureSubsystems() {
    drivetrain = buildDrivetrain();
    vision = buildVision();
    ledSubsystem = buildLedSubsystem();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(DrivetrainFactory.handleTeleopDrive(
      drivetrain, 
      controlboard::getThrottle, 
      controlboard::getStrafe, 
      controlboard::getRotation, 
      true
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}