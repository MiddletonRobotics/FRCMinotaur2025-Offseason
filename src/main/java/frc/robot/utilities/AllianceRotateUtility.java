package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utilities.constants.FieldConstants;

public class AllianceRotateUtility {
    public static class Mirrored {
        private static final Translation2d kFieldCenter = new Translation2d(FieldConstants.kFieldLength / 2.0, FieldConstants.kFieldWidth / 2.0);

        /**
         * Rotates an x coordinate to the correct side of the field based on the current alliance color.
         */
        public static double apply(double xCoordinate) {
            if (shouldRotate()) {
                return FieldConstants.kFieldLength - xCoordinate;
            } else {
                return xCoordinate;
            }
        }

        /** Flips a translation to the correct side of the field based on the current alliance color. */
        public static Translation2d apply(Translation2d translation) {
            if (shouldRotate()) {
                return translation.rotateAround(kFieldCenter, Rotation2d.k180deg);
            } else {
                return translation;
            }
        }

        /** Rotates a rotation based on the current alliance color. */
        public static Rotation2d apply(Rotation2d rotation) {
            if (shouldRotate()) {
                return rotation.rotateBy(Rotation2d.k180deg);
            } else {
                return rotation;
            }
        }

        /** Rotates a pose to the correct side of the field based on the current alliance color. */
        public static Pose2d apply(Pose2d pose) {
            if (shouldRotate()) {
                return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
            } else {
                return pose;
            }
        }

        /**
         * Rotates a translation3d to the correct side of the field based on the current alliance color.
         */
        public static Translation3d apply(Translation3d translation3d) {
            if (shouldRotate()) {
                var translation2d = apply(translation3d.toTranslation2d());
                return new Translation3d(translation2d.getX(), translation2d.getY(), translation3d.getZ());
            } else {
                return translation3d;
            }
        }

        public static boolean shouldRotate() {
            return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
        }
    }
}
