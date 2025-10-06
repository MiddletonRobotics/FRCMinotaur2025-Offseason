package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controlboard implements DriverControllerIO, OperatorControllerIO {
    private static Controlboard instance;

    public static synchronized Controlboard getInstance() {
        if(instance == null) {
            instance = new Controlboard();
        }

        return instance;
    }

    private XboxDriverController driverController;
    private XboxOperatorController operatorController;

    private Controlboard() {
        driverController = XboxDriverController.getInstance();
        operatorController = XboxOperatorController.getInstance();
    }

    @Override
    public double getThrottle() {
        return driverController.getThrottle();
    }

    @Override
    public double getStrafe() {
        return driverController.getStrafe();
    }

    @Override
    public double getRotation() {
        return driverController.getRotation();
    }

    @Override
    public double getRotationY() {
        return driverController.getRotationY();
    }

    @Override
    public Trigger resetGyro() {
        return driverController.resetGyro();
    }

     @Override
    public Trigger getWantToXWheels() {
        return driverController.getWantToXWheels();
    }

    @Override
    public Trigger getWantToAutoAlign() {
        return driverController.getWantToAutoAlign();
    }

    @Override
    public Trigger score() {
        return driverController.score();
    }

    @Override
    public Trigger scoreBarge() {
        return driverController.scoreBarge();
    }

    @Override
    public Trigger reefIntakeAlgae() {
        return driverController.reefIntakeAlgae();
    }

    @Override
    public Trigger bargeManualStage() {
        return driverController.bargeManualStage();
    }

    @Override
    public Trigger processorManualStage() {
        return driverController.processorManualStage();
    }

    @Override
    public Trigger stow() {
        return driverController.stow();
    }

    @Override
    public Trigger intakeFunnel() {
        return driverController.intakeFunnel();
    }

    @Override
    public Trigger exhaust() {
        return driverController.exhaust();
    }

    @Override
    public Trigger stageL1() {
        return driverController.stageL1();
    }

    @Override
    public Trigger stageL2() {
        return driverController.stageL2();
    }

    @Override
    public Trigger stageL3() {
        return driverController.stageL3();
    }

    @Override
    public Trigger stageL4() {
        return driverController.stageL4();
    }

    @Override
    public Trigger getCoralMode() {
        return driverController.getCoralMode();
    }

    @Override
    public Trigger getAlgaeMode() {
        return driverController.getAlgaeMode();
    }

    @Override
    public Trigger autoAlignLeft() {
        return driverController.autoAlignLeft();
    }

    @Override
    public Trigger autoAlignCenter() {
        return driverController.autoAlignCenter();
    }

    @Override
    public Trigger autoAlignRight() {
        return driverController.autoAlignRight();
    }

    @Override
    public Trigger autoAlignFeeder() {
        return driverController.autoAlignFeeder();
    }

    @Override
    public Trigger autoAlignProcessor() {
        return driverController.autoAlignProcessor();
    }

    @Override
    public Trigger autoAlignBarge() {
        return driverController.autoAlignBarge();
    }

    @Override
    public Trigger manualIntakeAlgae() {
        return driverController.manualIntakeAlgae();
    }

    @Override
    public Trigger leftStick() {
        return driverController.leftStick();
    }

    @Override
    public Trigger rightStick() {
        return driverController.rightStick();
    }

    @Override
    public Trigger povUp() {
        return driverController.povUp();
    }

    @Override
    public Trigger povDown() {
        return driverController.povDown();
    }

    @Override
    public Trigger povLeft() {
        return driverController.povLeft();
    }

    @Override
    public Trigger povRight() {
        return driverController.povRight();
    }

    @Override
    public Trigger setDefaultRobotWide() {
        return driverController.setDefaultRobotWide();
    }

    @Override
    public Trigger setDefaultRobotTight() {
        return driverController.setDefaultRobotTight();
    }

    @Override
    public void rumble(boolean intensity) {
        driverController.rumble(intensity);
    }

    @Override
    public Trigger toggleFieldRobotCentric() {
        return operatorController.toggleFieldRobotCentric();
    }

    @Override
    public Trigger getCoralManualMode() {
        return operatorController.getCoralManualMode();
    }

    @Override
    public Trigger getAlgaeManualMode() {
        return operatorController.getAlgaeManualMode();
    }
}
