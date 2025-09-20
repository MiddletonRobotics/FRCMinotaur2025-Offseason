// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.minolib.RobotConfiguration;
import frc.robot.commands.TeleopSwerve;
import frc.robot.constants.DefaultRobotConfiguration;
import frc.robot.subsystems.drivetrain.DrivetrainIOCTRE;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class RobotContainer {
  private RobotConfiguration robotConfiguration;
  private DrivetrainSubsystem drivetrain;
  private TeleopSwerve driveCommand;

  private CommandXboxController driverController;

  public RobotContainer() {
    robotConfiguration = new DefaultRobotConfiguration();

    configureControllers();
    configureSubsystems();
    configureBindings();
  }

  private void configureSubsystems() {
    drivetrain = new DrivetrainSubsystem(new DrivetrainIOCTRE());

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