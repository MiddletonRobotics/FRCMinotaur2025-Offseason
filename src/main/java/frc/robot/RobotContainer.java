// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.io.DriverButtonBindings;
import frc.robot.io.TestingButtonBindings;
import frc.robot.subsystems.drive.Drivetrain;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOReplay;
import frc.robot.subsystems.drive.ModuleIOReplay;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.constants.GlobalConstants;

public class RobotContainer {
  private Drivetrain drivetrain;

  private DriverButtonBindings driverController;
  private TestingButtonBindings testingController;

  public RobotContainer() {
    configureSubsystems();
    configureControllers();
    configureBindings();
  }

  private void configureSubsystems() {
    switch(GlobalConstants.kCurrentMode) {
      case REAL:
        drivetrain = new Drivetrain(
          new GyroIOPigeon2(), 
          new ModuleIOTalonFX(DrivetrainConstants.kCompetitionModuleConfiguration[0]), 
          new ModuleIOTalonFX(DrivetrainConstants.kCompetitionModuleConfiguration[1]), 
          new ModuleIOTalonFX(DrivetrainConstants.kCompetitionModuleConfiguration[2]), 
          new ModuleIOTalonFX(DrivetrainConstants.kCompetitionModuleConfiguration[3])
        );

        break;
      case SIM:
        drivetrain = new Drivetrain(
          new GyroIOPigeon2(), 
          new ModuleIOSim(), 
          new ModuleIOSim(), 
          new ModuleIOSim(), 
          new ModuleIOSim()
        );

        break;
      case REPLAY:
        drivetrain = new Drivetrain(
          new GyroIOReplay(), 
          new ModuleIOReplay(), 
          new ModuleIOReplay(), 
          new ModuleIOReplay(), 
          new ModuleIOReplay()
        );

        break;
    }
  }

  private void configureControllers() {
    driverController = new DriverButtonBindings(0);
    testingController = new TestingButtonBindings(2);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(DriveCommands.teleopDrive(
      drivetrain, 
      driverController::getForward, 
      driverController::getStrafe, 
      driverController::getTurn, 
      () -> false
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
