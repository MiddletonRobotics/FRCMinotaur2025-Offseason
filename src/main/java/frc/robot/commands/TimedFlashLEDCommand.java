package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.leds.LedSubsystem;

public class TimedFlashLEDCommand extends Command {
    LedSubsystem ledSubsystem;
    Superstructure superstructure;
    private Timer timer;
    private LedSubsystem.WantedState wantedState;
    private double timeToRun;

    public TimedFlashLEDCommand(Superstructure superstructure, LedSubsystem ledSubsystem, LedSubsystem.WantedState wantedState, double timeToRun) {
        this.superstructure = superstructure;
        this.ledSubsystem = ledSubsystem;
        timer = new edu.wpi.first.wpilibj.Timer();
        this.wantedState = wantedState;
        this.timeToRun = timeToRun;
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        superstructure.setExternalCommandAllowedToControlLEDs(true);
        timer.restart();
        ledSubsystem.setWantedAction(wantedState);
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(timeToRun)) {
            ledSubsystem.setWantedAction(LedSubsystem.WantedState.DISPLAY_OFF);
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.setExternalCommandAllowedToControlLEDs(false);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}