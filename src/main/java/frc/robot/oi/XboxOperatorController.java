package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.minolib.controller.CommandSimulatedXboxController;
import frc.robot.Robot;
import frc.robot.constants.ControllerConstants;

public class XboxOperatorController implements OperatorControllerIO {
    private final CommandXboxController controller;
    private static XboxOperatorController instance;

    public static synchronized XboxOperatorController getInstance() {
        if(instance == null) {
            instance = new XboxOperatorController();
        }

        return instance;
    }

    private XboxOperatorController() {
        if(Robot.isSimulation()) {
            controller = new CommandSimulatedXboxController(ControllerConstants.kDriverControllerPort);
        } else {
            controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);
        }
    }

    @Override
    public Trigger toggleFieldRobotCentric() {
        return controller.a();
    }

    @Override
    public Trigger getCoralManualMode() {
        return controller.start().and(controller.back().negate()).debounce(0.1);
    }

    @Override
    public Trigger getAlgaeManualMode() {
        return controller.back().and(controller.start().negate()).debounce(0.1);
    }
}
