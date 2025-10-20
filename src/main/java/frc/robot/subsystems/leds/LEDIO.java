package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LedState;

import org.littletonrobotics.junction.AutoLog;

public interface LedIO {
    @AutoLog
    public class LedInputs {}

    LedState getCurrentState();

    default void setLEDs(LedState wantedState) {}

    default void setLEDs(LedState[] wantedStates) {}

    default void clearAnimation() {}
}
