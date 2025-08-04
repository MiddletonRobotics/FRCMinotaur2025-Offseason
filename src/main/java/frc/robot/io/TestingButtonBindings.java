package frc.robot.io;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TestingButtonBindings {
    private CommandXboxController controller;

    public TestingButtonBindings(int port) {
        controller = new CommandXboxController(port);
    }

    public Trigger toggleTestingMode() {
        return controller.a();
    }

    public Trigger autoAutoscoreLeft() {
        return controller.x();
    }

    public Trigger autoAutoscoreRight() {
        return controller.b();
    }

    public Trigger autoCoralIntake() {
        return controller.rightBumper();
    }

    public Trigger autoLeft() {
        return controller.povLeft();
    }

    public Trigger autoRight() {
        return controller.povRight();
    }

    public Trigger incrementCoralScored() {
        return controller.povUp();
    }

    public Trigger decrementCoralScored() {
        return controller.povDown();
    }
}