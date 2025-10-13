package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControllerIO {
    double getThrottle();

    double getStrafe();

    double getRotation();

    double getRotationY();

    Trigger resetGyro();

    Trigger getWantToXWheels();

    Trigger getWantToAutoAlign();

    Trigger score();

    Trigger scoreBarge();

    Trigger stow();

    Trigger intakeFunnel();

    Trigger exhaust();

    Trigger stageL1();

    Trigger stageL2();

    Trigger stageL3();

    Trigger stageL4();

    Trigger getCoralMode();

    Trigger getAlgaeMode();

    Trigger autoAlignLeft();

    Trigger autoAlignCenter();

    Trigger autoAlignRight();

    Trigger autoAlignFeeder();

    Trigger autoAlignProcessor();

    Trigger autoAlignBarge();

    Trigger reefIntakeAlgae();

    Trigger manualIntakeAlgae();

    Trigger bargeManualStage();

    Trigger processorManualStage();

    Trigger leftStick();

    Trigger rightStick();

    Trigger povUp();

    Trigger povDown();

    Trigger povLeft();

    Trigger povRight();

    Trigger setDefaultRobotWide();

    Trigger setDefaultRobotTight();

    void rumble(boolean intensity);
}
