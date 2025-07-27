package frc.robot.utilities.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.LoggedTunableNumber;

public class VisionConstants {
    public static final LoggedTunableNumber kUseVision = new LoggedTunableNumber("Use Vision", 1);
    
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final double kAprilTagWidth = Units.inchesToMeters(6.5);
}
