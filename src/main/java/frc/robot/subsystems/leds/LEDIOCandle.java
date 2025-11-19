package frc.robot.subsystems.leds;

import frc.robot.constants.LEDConstants;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.google.flatbuffers.Constants;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.util.Color;

public class LedIOCANdle implements LedIO {
    private static final int COLOR_SCALAR = 255;
    private final CANdle candle;

    public LedIOCANdle(int port, String canBus) {
        this.candle = new CANdle(port, canBus);
        candle.configLEDType(CANdle.LEDStripType.GRBW);
    }

    @Override
    public void setLEDs(Color color) {
        candle.setLEDs((int) (color.red * COLOR_SCALAR), (int) (color.green * COLOR_SCALAR), (int) (color.blue * COLOR_SCALAR));
    }

    public void setLEDs(int red, int green, int blue) {
        candle.setLEDs(red, green, blue);
    }

    @Override
    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }

    @Override
    public void clearAnimation() {
        candle.clearAnimation(0);
    }
}
