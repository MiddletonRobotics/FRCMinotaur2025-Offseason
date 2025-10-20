package frc.robot.constants;

import frc.minolib.hardware.CANDeviceID;

public class LEDConstants {
    public static final CANDeviceID kCANdleId = new CANDeviceID(23, GlobalConstants.kCanivoreName);
    public static final int kNonCandleLEDCount = 68;
    public static final int kCandleLEDCount = 8;
    public static final int kMaxLEDCount = kNonCandleLEDCount + kCandleLEDCount;
    public static final double kLowBatteryThresholdVolts = 12.3;
}
