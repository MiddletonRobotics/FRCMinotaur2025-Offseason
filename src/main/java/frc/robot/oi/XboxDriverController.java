package frc.robot.oi;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.minolib.controller.CommandSimulatedXboxController;
import frc.robot.Robot;
import frc.robot.constants.ControllerConstants;

public class XboxDriverController implements DriverControllerIO {
    private final CommandXboxController controller;
    private static XboxDriverController instance;

    public static synchronized XboxDriverController getInstance() {
        if(instance == null) {
            instance = new XboxDriverController();
        }

        return instance;
    }

    private XboxDriverController() {
        if(Robot.isSimulation()) {
            controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);
        } else {
            controller = new CommandXboxController(ControllerConstants.kDriverControllerPort);
        }
    }

    @Override
    public double getThrottle() {
        return -(Math.pow(Math.abs(controller.getLeftY()), 1.5)) * Math.signum(controller.getLeftY());
    }

    @Override
    public double getStrafe() {
        return -(Math.pow(Math.abs(controller.getLeftX()), 1.5)) * Math.signum(controller.getLeftX());
    }

    @Override
    public double getRotation() {
        return -(Math.pow(Math.abs(controller.getRightX()), 2.0)) * Math.signum(controller.getRightX());
    }

    @Override
    public double getRotationY() {
        return -(Math.pow(Math.abs(controller.getRightY()), 2.0)) * Math.signum(controller.getRightY());
    }

    @Override
    public Trigger resetGyro() {
        return controller.back().and(controller.start().negate());
    }

     @Override
    public Trigger getWantToXWheels() {
        return controller.start().and(controller.back().negate());
    }

    @Override
    public Trigger getWantToAutoAlign() {
        return controller.start();
    }

    @Override
    public Trigger score() {
        return controller.rightTrigger();
    }

    @Override
    public Trigger scoreBarge() {
        return controller.b();
    }

    @Override
    public Trigger reefIntakeAlgae() {
        return controller.a();
    }

    @Override
    public Trigger bargeManualStage() {
        return controller.y();
    }

    @Override
    public Trigger processorManualStage() {
        return controller.x();
    }

    @Override
    public Trigger stow() {
        return controller.leftStick();
    }

    @Override
    public Trigger intakeFunnel() {
        return controller.leftTrigger();
    }

    @Override
    public Trigger exhaust() {
        return controller.povRight();
    }

    @Override
    public Trigger stageL1() {
        return controller.x();
    }

    @Override
    public Trigger stageL2() {
        return controller.a();
    }

    @Override
    public Trigger stageL3() {
        return controller.b();
    }

    @Override
    public Trigger stageL4() {
        return controller.y();
    }

    @Override
    public Trigger getCoralMode() {
        return controller.start().and(controller.back().negate()).debounce(0.1);
    }

    @Override
    public Trigger getAlgaeMode() {
        return controller.back().and(controller.start().negate()).debounce(0.1);
    }

    @Override
    public Trigger autoAlignLeft() {
        return controller.leftBumper();
    }

    @Override
    public Trigger autoAlignCenter() {
        return controller.leftBumper();
    }

    @Override
    public Trigger autoAlignRight() {
        return controller.rightBumper();
    }

    @Override
    public Trigger autoAlignFeeder() {
        return controller.rightStick().debounce(ControllerConstants.kPOVDebounceTimeSeconds.in(Seconds));
    }

    @Override
    public Trigger autoAlignProcessor() {
        return controller.povDown();
    }

    @Override
    public Trigger autoAlignBarge() {
        return controller.povUp();
    }

    @Override
    public Trigger manualIntakeAlgae() {
        return controller.rightStick();
    }

    @Override
    public Trigger leftStick() {
        return controller.leftStick();
    }

    @Override
    public Trigger rightStick() {
        return controller.rightStick();
    }

    @Override
    public Trigger povUp() {
        return controller.povUp();
    }

    @Override
    public Trigger povDown() {
        return controller.povDown();
    }

    @Override
    public Trigger povLeft() {
        return controller.povLeft();
    }

    @Override
    public Trigger povRight() {
        return controller.povRight();
    }

    @Override
    public void rumble(boolean intensity) {
        controller.getHID().setRumble(RumbleType.kBothRumble, intensity ? 1 : 0);
    }

    @Override
    public Trigger setDefaultRobotWide() {
        return controller.povDown();
    }

    @Override
    public Trigger setDefaultRobotTight() {
        return controller.povUp();
    }

}
