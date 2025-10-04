package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorControllerIO {
    Trigger toggleFieldRobotCentric();

    Trigger getCoralManualMode();

    Trigger getAlgaeManualMode();
}
