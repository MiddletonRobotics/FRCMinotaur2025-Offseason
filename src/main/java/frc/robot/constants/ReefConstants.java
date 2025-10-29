package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReefConstants {
    public enum ReefFaces {
        AB,
        CD,
        EF,
        GH,
        IJ,
        KL
    }

    public enum AlgaeIntakeLocation {
        L2,
        L3
    }

    public static final class AlgaeIntakeMapping {
        public final AlgaeIntakeLocation FRONT;
        public final AlgaeIntakeLocation BACK;

        public AlgaeIntakeMapping(AlgaeIntakeLocation front, AlgaeIntakeLocation back) {
            FRONT = front;
            BACK = back;
        }
    }

    public static final class ScoringCoralMappingRotationToTagID {
        public final int FRONT_ID;
        public final int BACK_ID;

        public ScoringCoralMappingRotationToTagID(int frontID, int backID) {
            FRONT_ID = frontID;
            BACK_ID = backID;
        }
    }

    /*

    public static final Map<Pose2d, Integer> blueAlliancePoseToTagIDsMap = Map.of(
        FieldConstants.getTagPose(21).toPose2d(), 21,
        FieldConstants.getTagPose(20).toPose2d(), 20,
        FieldConstants.getTagPose(19).toPose2d(), 19,
        FieldConstants.getTagPose(18).toPose2d(), 18,
        FieldConstants.getTagPose(17).toPose2d(), 17,
        FieldConstants.getTagPose(22).toPose2d(), 22
    );

    public static final Map<Pose2d, Integer> redAlliancePoseToTagIDsMap = Map.of(
        FieldConstants.getTagPose(6).toPose2d(), 6,
        FieldConstants.getTagPose(7).toPose2d(), 7,
        FieldConstants.getTagPose(8).toPose2d(), 8,
        FieldConstants.getTagPose(9).toPose2d(), 9,
        FieldConstants.getTagPose(10).toPose2d(), 10,
        FieldConstants.getTagPose(11).toPose2d(), 11
    );

    */

    public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> redAllianceAngleToTagIDsMap = Map.of(
        Rotation2d.fromDegrees(-60), new ScoringCoralMappingRotationToTagID(9, 6),
        Rotation2d.fromDegrees(-120), new ScoringCoralMappingRotationToTagID(8, 11),
        Rotation2d.k180deg, new ScoringCoralMappingRotationToTagID(7, 10),
        Rotation2d.fromDegrees(120), new ScoringCoralMappingRotationToTagID(6, 9),
        Rotation2d.fromDegrees(60), new ScoringCoralMappingRotationToTagID(11, 8),
        Rotation2d.kZero, new ScoringCoralMappingRotationToTagID(10, 7));

    public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> blueAllianceAngleToTagIDsMap = Map.of(
        Rotation2d.fromDegrees(-60), new ScoringCoralMappingRotationToTagID(19, 22),
        Rotation2d.fromDegrees(-120), new ScoringCoralMappingRotationToTagID(20, 17),
        Rotation2d.k180deg, new ScoringCoralMappingRotationToTagID(21, 18),
        Rotation2d.fromDegrees(120), new ScoringCoralMappingRotationToTagID(22, 19),
        Rotation2d.fromDegrees(60), new ScoringCoralMappingRotationToTagID(17, 20),
        Rotation2d.kZero, new ScoringCoralMappingRotationToTagID(18, 21)
    );

    public static final Map<Rotation2d, AlgaeIntakeMapping> redAllianceAlgae = Map.of(
        Rotation2d.fromDegrees(0), new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(60), new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(120), new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(180), new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(-120), new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(-60), new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2)
    );

    public static final Map<Rotation2d, AlgaeIntakeMapping> blueAllianceAlgae = Map.of(
        Rotation2d.fromDegrees(0), new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(60), new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(120), new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(180), new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3),
        Rotation2d.fromDegrees(-120), new AlgaeIntakeMapping(AlgaeIntakeLocation.L3, AlgaeIntakeLocation.L2),
        Rotation2d.fromDegrees(-60), new AlgaeIntakeMapping(AlgaeIntakeLocation.L2, AlgaeIntakeLocation.L3)
    );

    public static final Map<ReefFaces, Integer> redAllianceReefFacesToIds = Map.of(
        ReefFaces.AB, 7,
        ReefFaces.CD, 8,
        ReefFaces.EF, 9,
        ReefFaces.GH, 10,
        ReefFaces.IJ, 11,
        ReefFaces.KL, 6
    );

    public static final Map<ReefFaces, Integer> blueAllianceReefFacesToIds = Map.of(
        ReefFaces.AB, 18,
        ReefFaces.CD, 17,
        ReefFaces.EF, 22,
        ReefFaces.GH, 21,
        ReefFaces.IJ, 20,
        ReefFaces.KL, 19
    );
}