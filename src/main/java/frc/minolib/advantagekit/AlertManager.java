package frc.minolib.advantagekit;

import edu.wpi.first.wpilibj.Alert;

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

    /* Need to fix this to register all alerts either in a RobotState or Robot

    if (anyActive) {
        Robot.getInstance().triggerAlert();
    } else {
        RobotState.getInstance().cancelAlert();
    }

      */
  }
}