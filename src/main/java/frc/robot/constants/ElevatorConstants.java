package frc.robot.constants;

import edu.wpi.first.math.util.Units;
import frc.minolib.hardware.CANDeviceID;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.PIDConfiguration;

public class ElevatorConstants {
    public static final CANDeviceID leftMotorID = new CANDeviceID(15, GlobalConstants.kCanivoreName);
    public static final CANDeviceID rightMotorID = new CANDeviceID(16, GlobalConstants.kCanivoreName);
    public static final double sprocketPitchDiameter = Units.inchesToMeters(1.273); // 16T #25
    public static final MechanismRatio motorRatio = new MechanismRatio(10, 60, Math.PI * sprocketPitchDiameter); // From CAD, not sure if final
    public static final boolean leftMotorInvert = true;
    public static final boolean rightMotorInvert = false;

    public static final int positionSlot = 0;
    public static final PIDConfiguration elevatorPIDConfig = new PIDConfiguration(2.0, 0, 0.1, 0, 0.115, 0.003, 0.39);
    public static final double maxVelocity = 2.0; // m/s
    public static final double maxGentleVelocity = 1.0; // rad/s
    public static final double maxAcceleration = 12.0; // m/s
    public static final double maxJerk = 0.0; // m/s/s
    public static final double maxGentleAcceleration = 3.0; // m/s

    public static final double rotorBootOffset = -0.125;
    public static final double minHeight = Units.inchesToMeters(0);
    public static final double maxHeight = Units.inchesToMeters(27);
    public static final double startingHeight = minHeight;
    public static final double stowHeight = Units.inchesToMeters(2.0);
    public static final double stowAlgaeHeight = Units.inchesToMeters(11.0);
    public static final double unstowAlgaeHeight = Units.inchesToMeters(13.0);
    public static final double scoreAlgaeHeight = Units.inchesToMeters(5.0);

    public static final double levelOneHeight = Units.inchesToMeters(4.0);
    public static final double levelTwoHeight = Units.inchesToMeters(11.0);
    public static final double levelThreeHeight = Units.inchesToMeters(26.0);
    public static final double levelFourHeight = Units.inchesToMeters(27.0);

    public static final double netHeight = maxHeight;
    public static final double lowAlgaeHeight = Units.inchesToMeters(10.0);
    public static final double highAlgaeHeight = Units.inchesToMeters(7.0);

    // For simulatilon.
    public static final double simCarriageMass = Units.lbsToKilograms(30.0);

    public enum ElevatorHeight {
        RETRACTED(minHeight),
        L1(levelOneHeight),
        L2(levelTwoHeight),
        L2_ALGAE(lowAlgaeHeight),
        L3(levelThreeHeight),
        L3_ALGAE(highAlgaeHeight),
        L4(levelFourHeight),
        BARGE(maxHeight);

        private double position;

        private ElevatorHeight(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }
}
