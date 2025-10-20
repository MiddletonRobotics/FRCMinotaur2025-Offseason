package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.LEDConstants;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LedSubsystem extends SubsystemBase {
    private final LedIO io;

    public record PercentageSetpoint(double pct, LedState color) {}

    public LedSubsystem(final LedIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        super.periodic();
        Logger.recordOutput("LED/currentCommand", (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
    }

    public LedState getCurrentState() {
        return io.getCurrentState();
    }

    /* change runOnce to run in case we have to keep setting the LED color periodically? */
    public Command commandSolidColor(LedState wantedState) {
        return run(() -> setSolidColor(wantedState)).ignoringDisable(true).withName("LED Solid Color");
    }

    public Command commandSolidColor(Supplier<LedState> wantedState) {
        return run(() -> setSolidColor(wantedState.get())).ignoringDisable(true).withName("LED Solid Color");
    }

    public Command commandSolidPattern(LedState[] wantedStates) {
        return run(() -> setSolidPattern(wantedStates)).ignoringDisable(true).withName("LED Solid Pattern");
    }

    public Command commandPercentageFull(DoubleSupplier percentageFull, LedState wantedState) {
        return run(() -> setPercentageFull(percentageFull.getAsDouble(), wantedState)).ignoringDisable(true);
    }

    public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
        return run(() -> setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color)).ignoringDisable(true);
    }

    public Command commandBlinkingState(LedState wantedStateOne, LedState wantedStateTwo, double durationOne, double durationTwo) {
        return new SequentialCommandGroup(
            Commands.runOnce(() -> setSolidColor(wantedStateOne)),
            new WaitCommand(durationOne),
            Commands.runOnce(() -> setSolidColor(wantedStateTwo)),
            new WaitCommand(durationTwo)
        ).repeatedly().ignoringDisable(true).withName("Blinking LED command");
    }

    public Command commandBlinkingStateWithoutScheduler(LedState wantedStateOne, LedState wantedStateTwo, double durationOne, double durationTwo) {
        var state = new Object() {
            public boolean color1 = true;
            public double timestamp = Timer.getFPGATimestamp();
        };

        return Commands.runOnce(() -> {
            state.color1 = true;
            state.timestamp = Timer.getFPGATimestamp();
        }).andThen(commandSolidColor(() -> {
            if (state.color1 && state.timestamp + durationOne <= Timer.getFPGATimestamp()) {
                state.color1 = false;
                state.timestamp = Timer.getFPGATimestamp();
            } else if (!state.color1 && state.timestamp + durationTwo <= Timer.getFPGATimestamp()) {
                state.color1 = true;
                state.timestamp = Timer.getFPGATimestamp();
            }

            if (state.color1) {
                return wantedStateOne;
            } else {
                return wantedStateTwo;
            }
        })).ignoringDisable(true).withName("Blinking LED command");
    }

    public Command commandBlinkingState(LedState wantedStateOne, LedState wantedStateTwo, double duration) {
        return commandBlinkingState(wantedStateOne, wantedStateTwo, duration, duration).ignoringDisable(true);
    }

    private void setSolidColor(LedState wantedState) {
        io.setLEDs(wantedState);
    }

    private void setSolidPattern(LedState[] wantedStates) {
        io.setLEDs(wantedStates);
    }

    private void setPercentageFull(double percentageFull, LedState wantedState) {
        LedState[] pixels = new LedState[LEDConstants.kMaxLEDCount / 2];
        for (int i = 0; i < pixels.length; i++) {
            if (i < pixels.length * Math.min(1.0, Math.max(0.0, percentageFull))) {
                pixels[i] = wantedState;
            }
        }
    }

    @SuppressWarnings("unused")
    private LedState[] mirror(LedState[] wantedStates) {
        LedState[] fullPixels = new LedState[LEDConstants.kMaxLEDCount];

        for (int i = LEDConstants.kCandleLEDCount; i < LEDConstants.kCandleLEDCount + (LEDConstants.kNonCandleLEDCount / 2); i++) {
            fullPixels[fullPixels.length - i - 2] = wantedStates[i];
            fullPixels[i] = wantedStates[i];
        }

        return fullPixels;
    }
}