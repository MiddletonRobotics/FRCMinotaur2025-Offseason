package frc.minolib.wpilib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class SysIDRoutineChooser {
  private static final SysIDRoutineChooser instance = new SysIDRoutineChooser();

  private Map<Integer, Command> dynamicForwardRoutines = new HashMap<>();
  private Map<Integer, Command> dynamicReverseRoutines = new HashMap<>();
  private Map<Integer, Command> quasistaticForwardRoutines = new HashMap<>();
  private Map<Integer, Command> quasistaticReverseRoutines = new HashMap<>();
  private Command dynamicForwardSelectorCommand = null;
  private Command dynamicBackupSelectorCommand = null;
  private Command quasistaticForwardSelectorCommand = null;
  private Command quasistaticBackwardSelectorCommand = null;

  private final LoggedDashboardChooser<Integer> sysIdChooser = new LoggedDashboardChooser<>("SysIdRoutineChooser");

  private SysIDRoutineChooser() {}

  public static SysIDRoutineChooser getInstance() {
    return instance;
  }

  public void addOption(String name, SysIdRoutine sysIdRoutine) {
    sysIdChooser.addOption(name, name.hashCode());
    dynamicForwardRoutines.put(name.hashCode(), sysIdRoutine.dynamic(Direction.kForward));
    dynamicReverseRoutines.put(name.hashCode(), sysIdRoutine.dynamic(Direction.kReverse));
    quasistaticForwardRoutines.put(name.hashCode(), sysIdRoutine.quasistatic(Direction.kForward));
    quasistaticReverseRoutines.put(name.hashCode(), sysIdRoutine.quasistatic(Direction.kReverse));
  }

  public Command getDynamicForward() {
    if (dynamicForwardSelectorCommand == null) {
      dynamicForwardSelectorCommand = new SelectCommand<>(this.dynamicForwardRoutines, sysIdChooser::get);
    }

    return dynamicForwardSelectorCommand;
  }

  public Command getDynamicReverse() {
    if (dynamicBackupSelectorCommand == null) {
      dynamicBackupSelectorCommand = new SelectCommand<>(this.dynamicReverseRoutines, sysIdChooser::get);
    }

    return dynamicBackupSelectorCommand;
  }

  public Command getQuasistaticForward() {
    if (quasistaticForwardSelectorCommand == null) {
      quasistaticForwardSelectorCommand = new SelectCommand<>(this.quasistaticForwardRoutines, sysIdChooser::get);
    }

    return quasistaticForwardSelectorCommand;
  }

  public Command getQuasistaticReverse() {
    if (quasistaticBackwardSelectorCommand == null) {
      quasistaticBackwardSelectorCommand = new SelectCommand<>(this.quasistaticReverseRoutines, sysIdChooser::get);
    }

    return quasistaticBackwardSelectorCommand;
  }
}