package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverButtonBindings implements DriverControls {
  private CommandXboxController controller;

  public DriverButtonBindings(int port) {
    controller = new CommandXboxController(port);
  }

  @Override
  public double getForward() {
    return -controller.getLeftY();
  }

  @Override
  public double getStrafe() {
    return -controller.getLeftX();
  }

  @Override
  public double getTurn() {
    return -controller.getRightX();
  }

  @Override
  public Trigger resetFieldCentric() {
    return new Trigger(() -> false);
  }

  @Override
  public Trigger coralIntake() {
    return controller.leftBumper();
  }

  @Override
  public Trigger coralOuttake() {
    return controller.rightBumper();
  }

  @Override
  public Trigger setLocationL1() {
    return controller.povUp();
  }

  @Override
  public Trigger setLocationL2() {
    return controller.povRight();
  }

  @Override
  public Trigger setLocationL3() {
    return controller.povDown();
  }

  @Override
  public Trigger setLocationL4() {
    return controller.povLeft();
  }

  @Override
  public Trigger autoscoreLeft() {
    return controller.x();
  }

  @Override
  public Trigger autoscoreRight() {
    return controller.b();
  }

  @Override
  public Trigger manualScore() {
    return controller.y();
  }

  @Override
  public Trigger climb() {
    return controller.a();
  }

  @Override
  public Trigger otbMagic() {
    return controller.rightTrigger(0.1);
  }

  @Override
  public Trigger algaeDescore() {
    return controller.leftTrigger(0.1);
  }

  @Override
  public Trigger zeroElevator() {
    return controller.back();
  }

  @Override
  public Trigger toggleVision() {
    return controller.start();
  }

  @Override
  public Trigger coralEject() {
    return controller.rightStick();
  }

  @Override
  public Trigger toggleOtbRunthrough() {
    return controller.leftStick();
  }

  @Override
  public Trigger zeroClimb() {
    return new Trigger(() -> false);
  }
}