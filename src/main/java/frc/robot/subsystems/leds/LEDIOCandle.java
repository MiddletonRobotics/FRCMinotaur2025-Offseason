package frc.robot.subsystems.leds;

import frc.robot.constants.LEDConstants;
import frc.robot.subsystems.leds.LedState;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.google.flatbuffers.Constants;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color;

public class LedIOCANdle implements LedIO {
    private final CANdle candle;
    private LedState currentState = LedState.kRed;
    private LedState[] currentPixels = new LedState[LEDConstants.kMaxLEDCount];

    public LedIOCANdle() {
        this.candle = new CANdle(LEDConstants.kCANdleId.deviceNumber, LEDConstants.kCANdleId.CANbusName);
        candle.configBrightnessScalar(1.0);
        candle.configLEDType(CANdle.LEDStripType.RGBW);
    }

    public LedState getCurrentState() {
        return currentState;
    }

    public LedState[] getCurrentPixels() {
        return currentPixels;
    }

    @Override
    public void setLEDs(LedState wantedState) {
        if(wantedState == null) currentState = LedState.kOff;
        currentState = wantedState;

        candle.setLEDs(currentState.red, currentState.green, currentState.blue);
    }

    @Override
    public void setLEDs(LedState[] wantedStates) {
        if (wantedStates == null || wantedStates.length == 0) {
            return;
        }

        LedState run = wantedStates[0];
        int runStart = 0;

        for (int i = 0; i < wantedStates.length; i++) {
            if (wantedStates[i] == null) wantedStates[i] = LedState.kOff;
            if (!run.equals(wantedStates[i])) {
                if (candle != null) candle.setLEDs(run.red, run.green, run.blue, 255, runStart, i - runStart);
                runStart = i;
                run = wantedStates[i];
                currentPixels[i] = run;
            }
        }

        candle.setLEDs(run.red, run.green, run.blue, 255, runStart, wantedStates.length - runStart);
    }

    @Override
    public void clearAnimation() {
        candle.clearAnimation(0);
    }
}
