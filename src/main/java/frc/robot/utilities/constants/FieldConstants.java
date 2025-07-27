package frc.robot.utilities.constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.google.flatbuffers.FlexBuffers.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public static final class FieldConstants {
    public static final FieldType kFieldType = FieldType.WELDED;

    public static final double kFieldLength = VisionConstants.kAprilTagLayout.getFieldLength();
    public static final double kFieldWidth = VisionConstants.kAprilTagLayout.getFieldWidth();
    public static final double kStartingLineX = Units.inchesToMeters(299.438); // Measured from the inside of starting line
    public static final double kAlgaeDiameter = Units.inchesToMeters(16);
    public static final double kCoralDiameter = Units.inchesToMeters(4.5);

    public static class Processor {
        public static final Pose2d kCenterFace = new Pose2d(VisionConstants.kAprilTagLayout.getTagPose(16).get().getX(), 0, Rotation2d.fromDegrees(90));
        public static final Pose2d kOpposingCenterFace = new Pose2d(VisionConstants.kAprilTagLayout.getTagPose(3).get().getX(), kFieldWidth, Rotation2d.fromDegrees(-90));
    }

    public static class Barge {
        public static final double kNetWidth = Units.inchesToMeters(40.0);
        public static final double kNetHeight = Units.inchesToMeters(88.0);

        public static final double kCageWidth = Units.inchesToMeters(6.0);
        public static final Translation2d kFarCage = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
        public static final Translation2d kMiddleCage = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
        public static final Translation2d kCloseCage = new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

        // Measured from floor to bottom of cage
        public static final double kDeepHeight = Units.inchesToMeters(3.125);
        public static final double kShallowHeight = Units.inchesToMeters(30.125);
    }

    public static class CoralStation {
        public static final double kStationLength = Units.inchesToMeters(79.750);
        public static final Pose2d kRightCenterFace = new Pose2d(Units.inchesToMeters(33.526), Units.inchesToMeters(25.824), Rotation2d.fromDegrees(144.011 - 90));
        public static final Pose2d kLeftCenterFace = new Pose2d(kRightCenterFace.getX(), kFieldWidth - kRightCenterFace.getY(), Rotation2d.fromRadians(-kRightCenterFace.getRotation().getRadians()));
    }

    public static class Reef {
        public static final double kFaceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d kCenter = new Translation2d(Units.inchesToMeters(176.746), kFieldWidth / 2.0);
        public static final double kFaceToZoneLine = Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

        public static final Pose2d[] kCenterFaces = new Pose2d[6]; // Starting facing the driver station in clockwise order
        public static final List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
        public static final List<Map<ReefHeight, Pose2d>> branchPositions2d = new ArrayList<>();

      static {
        // Initialize faces
        var aprilTagLayout = VisionConstants.kAprilTagLayout;
        kCenterFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
        kCenterFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
        kCenterFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
        kCenterFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
        kCenterFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
        kCenterFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

        // Initialize branch positions
        for (int face = 0; face < 6; face++) {
            Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
            Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
            Map<ReefHeight, Pose2d> fillRight2d = new HashMap<>();
            Map<ReefHeight, Pose2d> fillLeft2d = new HashMap<>();

            for (var level : ReefHeight.values()) {
                Pose2d poseDirection = new Pose2d(kCenter, Rotation2d.fromDegrees(180 - (60 * face)));
                double adjustX = Units.inchesToMeters(30.738);
                double adjustY = Units.inchesToMeters(6.469);

                var rightBranchPose = new Pose3d(
                    new Translation3d(
                        poseDirection.transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero)).getX(),
                        poseDirection.transformBy(new Transform2d(adjustX, adjustY, Rotation2d.kZero)).getY(),
                        level.height
                    ),
                    new Rotation3d(0,Units.degreesToRadians(level.pitch), poseDirection.getRotation().getRadians())
                );

                var leftBranchPose = new Pose3d(
                    new Translation3d(
                        poseDirection.transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero)).getX(),
                        poseDirection.transformBy(new Transform2d(adjustX, -adjustY, Rotation2d.kZero)).getY(),
                        level.height
                    ),
                    new Rotation3d(0, Units.degreesToRadians(level.pitch), poseDirection.getRotation().getRadians())
                );

                fillRight.put(level, rightBranchPose);
                fillLeft.put(level, leftBranchPose);
                fillRight2d.put(level, rightBranchPose.toPose2d());
                fillLeft2d.put(level, leftBranchPose.toPose2d());
            }

            branchPositions.add(fillRight);
            branchPositions.add(fillLeft);
            branchPositions2d.add(fillRight2d);
            branchPositions2d.add(fillLeft2d);
        }
      }
    }

    public static class StagingPositions {
      // Measured from the center of the ice cream
      public static final double kSeparation = Units.inchesToMeters(72.0);
      public static final Translation2d[] kIceCreams = new Translation2d[3];

      static {
        for (int i = 0; i < 3; i++) {
          kIceCreams[i] =
              new Translation2d(
                  Units.inchesToMeters(48), kFieldWidth / 2.0 - kSeparation + kSeparation * i);
        }
      }
    }

    public enum ReefHeight {
      L1(0, Units.inchesToMeters(25.0), 0),
      L2(1, Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
      L3(2, Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
      L4(3, Units.inchesToMeters(72), -90);

      ReefHeight(int levelNumber, double height, double pitch) {
        this.levelNumber = levelNumber;
        this.height = height;
        this.pitch = pitch; // Degrees
      }

      public static ReefHeight fromLevel(int level) {
        return Arrays.stream(values())
            .filter(height -> height.ordinal() == level)
            .findFirst()
            .orElse(L4);
      }

      public final int levelNumber;
      public final double height;
      public final double pitch;
    }

    @RequiredArgsConstructor
    public enum FieldType {
      ANDYMARK,
      WELDED,
    }
  }
