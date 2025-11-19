package frc.robot.subsystems.leds;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj.util.Color;

public interface LedIO {
    @AutoLog
    public class LedInputs {}

    default void setLEDs(Color color) {}

    default void setLEDs(int red, int green, int blue) {}

    default void setAnimation(Animation animation) {}

    default void clearAnimation() {}
}
