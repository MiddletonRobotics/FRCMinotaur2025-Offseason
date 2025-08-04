package frc.robot.utilities;

import edu.wpi.first.wpilibj.Alert;
//import frc.robot.RobotState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AlertManager {
  private static List<Alert> alerts = new ArrayList<>();

  // this is a static class and cannot be instantiated without reflection
  private AlertManager() {}

  public static void registerAlert(Alert... alertsList) {
    alerts.addAll(Arrays.asList(alertsList));
  }

  /**
   * Updates all of the alerts in the manager. This must be called AFTER {@code
   * CommandScheduler.getInstance().run()} in the robot periodic to work properly.
   */
  public static void update() {
    boolean anyActive = false;
    for (Alert alert : alerts) {
      anyActive |= alert.get();
    }

    if (anyActive) {
      //RobotState.getInstance().triggerAlert();
    } else {
      //RobotState.getInstance().cancelAlert();
    }
  }
}