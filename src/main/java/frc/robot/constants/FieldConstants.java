package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

import frc.robot.utilities.Fiducials;

import java.util.Map;

public class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(12 * 57 + 6.875); // From sim
    public static final double fieldWidth = Units.inchesToMeters(12 * 26 + 5); // From sim

    // Reef measurements
    public static final double robotReefWallPrescoreClearanceDistance = Units.inchesToMeters(40.0);
    public static final double robotReefWallPrescoreOffsetDistance = Units.inchesToMeters(44.0);
    public static final double robotReefWallScoringOffsetDistance = Units.inchesToMeters(30.0);
    public static final double robotReefWallL1ScoringOffsetDistance = Units.inchesToMeters(28.0);
    public static final Rotation2d L1AngleOffset = Rotation2d.fromDegrees(0.0);
    public static final double robotAlgaeIntakeOffsetDistance = Units.inchesToMeters(36.0);
    public static final double robotReefWallL4ScoringOffsetDistance = Units.inchesToMeters(26.0);
    public static final double netScoringOffsetDistance = Units.inchesToMeters(12.0); // TBD
    public static final double processorScoringOffsetDistance = Units.inchesToMeters(24.0); // TBD
    public static final Transform2d tagToLeftReefTipTransform = new Transform2d(Units.inchesToMeters(-2.0), Units.inchesToMeters(-6.5), Rotation2d.kZero);
    public static final Transform2d tagToRightReefTipTransform = new Transform2d(Units.inchesToMeters(-2.0), Units.inchesToMeters(6.5), Rotation2d.kZero);
    public static final Pose2d reefCenterBlue =
        new Pose2d(Fiducials.aprilTagFiducials[20].getPose().toPose2d().getTranslation().plus(Fiducials.aprilTagFiducials[17].getPose().toPose2d().getTranslation()).div(2.0), Rotation2d.kZero);

    public static final Pose2d reefCenterRed =
        new Pose2d(Fiducials.aprilTagFiducials[9].getPose().toPose2d().getTranslation().plus(Fiducials.aprilTagFiducials[6].getPose().toPose2d().getTranslation()).div(2.0), Rotation2d.kZero);

    // Tag shennaniganery
    private static final Map<Integer, Integer> blueToRedMap = Map.ofEntries(
        Map.entry(20, 11),
        Map.entry(19, 6),
        Map.entry(18, 7),
        Map.entry(17, 8),
        Map.entry(22, 9),
        Map.entry(21, 10)
    );

    public static int getReefTagForAlliance(int tagID, boolean isBlue) {
        if (isBlue) {
            if (blueToRedMap.containsValue(tagID)) {
                for (Map.Entry<Integer, Integer> entry : blueToRedMap.entrySet()) {
                    if (entry.getValue() == tagID) {
                    return entry.getKey();
                    }
                }
            }
            return tagID;
        } else {
            if (blueToRedMap.containsKey(tagID)) {
                return blueToRedMap.get(tagID);
            }

            return tagID;
        }
    }
}
