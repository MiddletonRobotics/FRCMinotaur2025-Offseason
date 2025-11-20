// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.google.flatbuffers.Constants;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.command_factories.DrivetrainFactory;
import frc.robot.commands.TimedFlashLEDCommand;
import frc.robot.constants.BuildConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.minolib.advantagekit.LocalADStarAK;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.hardware.MinoCANBus;
import frc.minolib.phoenix.PhoenixUtility;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;
  private double autoStart;
  private boolean autoMessagePrinted;
  private MinoCANBus canivoreBus;

  //private WPILOGWriter writer = new WPILOGWriter();
  //private Alert writerAlert = new Alert("WPILOGWriter Failed to start", AlertType.kError);

  private final Timer disabledTimer = new Timer();
  private final Timer canInitialErrorTimer = new Timer();
  private final Timer canErrorTimer = new Timer();
  private final Timer canivoreErrorTimer = new Timer();

  private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
  private final Alert logReceiverQueueAlert = new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.kError);
  private final Alert lowBatteryAlert = new Alert("Battery voltage is very low, consider turning off the robot or replacing the battery.", AlertType.kWarning);
  private final Alert gitAlert = new Alert("Please wait to enable, JITing in progress.", AlertType.kWarning);
  private final Alert noAutoSelectedAlert = new Alert("No auto selected: please select an auto", AlertType.kWarning);

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (GlobalConstants.kCurrentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        break;
      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    Logger.registerURCL(URCL.startExternal()); // Start Unoffical REV Compatable Logger

    // Start AdvantageKit logger
    Logger.start();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    Pathfinding.setPathfinder(new LocalADStarAK());

    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
      String name = command.getName();
      int count = commandCounts.getOrDefault(name, 0) + (Boolean.TRUE.equals(active) ? 1 : -1);
      commandCounts.put(name, count);
      Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
      Logger.recordOutput("CommandsAll/" + name, count > 0);
    };

    CommandScheduler.getInstance().onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance().onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    if (GlobalConstants.kCurrentMode == GlobalConstants.Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
    }

    PathPlannerLogging.setLogCurrentPoseCallback(pose -> Logger.recordOutput("PathFollowing/currentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("PathFollowing/targetPose", pose));
    PathPlannerLogging.setLogActivePathCallback(poses -> Logger.recordOutput("PathFollowing/activePath", poses.toArray(new Pose2d[0])));

    canInitialErrorTimer.restart();
    canErrorTimer.restart();
    canivoreErrorTimer.restart();
    disabledTimer.restart();

    if (!GlobalConstants.kTuningMode) {
      try {
        Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
        watchdogField.setAccessible(true);
        Watchdog watchdog = (Watchdog) watchdogField.get(this);
        watchdog.setTimeout(0.2);
      } catch (Exception e) {
        DriverStation.reportWarning("Failed to disable loop overrun warnings", false);
      }
    }

    robotContainer = new RobotContainer();
    canivoreBus = new MinoCANBus(GlobalConstants.kCanivoreName);

    PathfindingCommand.warmupCommand().schedule();

    if (!GlobalConstants.kTuningMode) {
      Threads.setCurrentThreadPriority(true, 10);
    }

    Shuffleboard.getTab("Dashboard").add(DrivetrainFactory.resetDrivetrainPose(
      new InstantCommand(() -> robotContainer.getDrivetrainSubsystem().resetOdometry(new Pose2d())) { //TODO: Change the parameter to something when we actually run autonomous (maybe add simulation stuff here too)
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
      }.andThen(new TimedFlashLEDCommand(
        robotContainer.getSuperstructureSubsystem(),
        robotContainer.getLedSubsystem(),
        LedSubsystem.WantedState.DISPLAY_POSE_RESET,
        2.0
      )).andThen(new InstantCommand(() -> robotContainer.getSuperstructureSubsystem().toggleHasPoseBeenSetForPrematch(true)) {
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
      }))).withSize(6, 3);
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    PhoenixUtility.refreshAll();
    LoggedTracer.record("PhoenixRefresh");

    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");

    logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

    // Check CAN status
    var canStatus = RobotController.getCANStatus();
    Logger.recordOutput("CANStatus/OffCount", canStatus.busOffCount);
    Logger.recordOutput("CANStatus/TxFullCount", canStatus.txFullCount);
    Logger.recordOutput("CANStatus/ReceiveErrorCount", canStatus.receiveErrorCount);
    Logger.recordOutput("CANStatus/TransmitErrorCount", canStatus.transmitErrorCount);

    if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
      canErrorTimer.restart();
    }

    canErrorAlert.set(!canErrorTimer.hasElapsed(GlobalConstants.kCANErrorTimeThreshold) && canInitialErrorTimer.hasElapsed(GlobalConstants.kCANErrorTimeThreshold));

    canivoreBus.updateInputs();

    // Update low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }

    if (RobotController.getBatteryVoltage() < GlobalConstants.kLowBatteryVoltage && disabledTimer.hasElapsed(GlobalConstants.kLowBatteryDisabledTime)) {
      lowBatteryAlert.set(true);
    }

    // JIT alert
    gitAlert.set(isJITing());

    // Print auto duration
    if (autonomousCommand != null && !autonomousCommand.isScheduled() && !autoMessagePrinted) {
      if (DriverStation.isAutonomousEnabled()) {
        System.out.println(
            String.format("*** Auto finished in %.2f secs ***", Timer.getTimestamp() - autoStart));
      } else {
        System.out.println(
            String.format("*** Auto cancelled in %.2f secs ***", Timer.getTimestamp() - autoStart));
      }

      autoMessagePrinted = true;
    }

    //robotContainer.periodic();
    // Record cycle time
    LoggedTracer.record("RobotPeriodic");
  }

  /** Returns whether we should wait to enable because JIT optimizations are in progress. */
  public static boolean isJITing() {
    return Timer.getTimestamp() < 45.0;
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    //robotContainer.checkAllianceColor();

    //check for unselected autonomous
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    //robotContainer.checkAllianceColor();

    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;

    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    //robotContainer.autonomousInit();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    // robotContainer.teleopInit();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
    public void simulationPeriodic() {
        Logger.recordOutput("Simulation/CoralPoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput("Simulation/AlgaePoses", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
