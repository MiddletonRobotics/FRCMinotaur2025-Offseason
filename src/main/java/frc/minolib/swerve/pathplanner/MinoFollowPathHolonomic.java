package frc.minolib.swerve.pathplanner;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Follow a path using a PPHolonomicDriveController */
public class MinoFollowPathHolonomic extends MinoFollowPathCommand {
    /**
     * Construct a path following command that will use a holonomic drive controller
     * for holonomic
     * drive trains
     *
     * @param path                 The path to follow
     * @param poseSupplier         Function that supplies the current field-relative
     *                             pose of the robot
     * @param speedsSupplier       Function that supplies the current robot-relative
     *                             chassis speeds
     * @param outputRobotRelative  Function that will apply the robot-relative
     *                             output speeds of this
     *                             command
     * @param translationConstants PID constants for the translation PID controllers
     * @param rotationConstants    PID constants for the rotation controller
     * @param maxModuleSpeed       The max speed of a drive module in meters/sec
     * @param driveBaseRadius      The radius of the drive base in meters. For
     *                             swerve drive, this is the
     *                             distance from the center of the robot to the
     *                             furthest module. For mecanum, this is the
     *                             drive base width / 2
     * @param period               Period of the control loop in seconds, default is
     *                             0.02s
     * @param replanningConfig     Path replanning configuration
     * @param shouldFlipPath       Should the path be flipped to the other side of
     *                             the field? This will
     *                             maintain a global blue alliance origin.
     * @param requirements         Subsystems required by this command, usually just
     *                             the drive subsystem
     */
    public MinoFollowPathHolonomic(
        PathPlannerPath path,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier,
        Consumer<ChassisSpeeds> outputRobotRelative,
        PIDConstants translationConstants,
        PIDConstants rotationConstants,
        double translationKa,
        double maxModuleSpeed,
        double driveBaseRadius,
        double period,
        BooleanSupplier shouldFlipPath,
        Subsystem... requirements
    ) {
        super(
            path,
            poseSupplier,
            speedsSupplier,
            outputRobotRelative,
            new MinoHolonomicDriveController(path, translationConstants, rotationConstants, translationKa, period, maxModuleSpeed, driveBaseRadius),
            shouldFlipPath,
            requirements
        );
    }

    /**
     * Construct a path following command that will use a holonomic drive controller
     * for holonomic
     * drive trains
     *
     * @param path                 The path to follow
     * @param poseSupplier         Function that supplies the current field-relative
     *                             pose of the robot
     * @param speedsSupplier       Function that supplies the current robot-relative
     *                             chassis speeds
     * @param outputRobotRelative  Function that will apply the robot-relative
     *                             output speeds of this
     *                             command
     * @param translationConstants PID constants for the translation PID controllers
     * @param rotationConstants    PID constants for the rotation controller
     * @param maxModuleSpeed       The max speed of a drive module in meters/sec
     * @param driveBaseRadius      The radius of the drive base in meters. For
     *                             swerve drive, this is the
     *                             distance from the center of the robot to the
     *                             furthest module. For mecanum, this is the
     *                             drive base width / 2
     * @param replanningConfig     Path replanning configuration
     * @param shouldFlipPath       Should the path be flipped to the other side of
     *                             the field? This will
     *                             maintain a global blue alliance origin.
     * @param requirements         Subsystems required by this command, usually just
     *                             the drive subsystem
     */
    public MinoFollowPathHolonomic(
            PathPlannerPath path,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<ChassisSpeeds> outputRobotRelative,
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            double translationKa,
            double maxModuleSpeed,
            double driveBaseRadius,
            BooleanSupplier shouldFlipPath,
            Subsystem... requirements) {
        this(
                path,
                poseSupplier,
                speedsSupplier,
                outputRobotRelative,
                translationConstants,
                rotationConstants,
                translationKa,
                maxModuleSpeed,
                driveBaseRadius,
                0.02,
                shouldFlipPath,
                requirements);
    }

    /**
     * Construct a path following command that will use a holonomic drive controller
     * for holonomic
     * drive trains
     *
     * @param path                The path to follow
     * @param poseSupplier        Function that supplies the current field-relative
     *                            pose of the robot
     * @param speedsSupplier      Function that supplies the current robot-relative
     *                            chassis speeds
     * @param outputRobotRelative Function that will apply the robot-relative output
     *                            speeds of this
     *                            command
     * @param config              Holonomic path follower configuration
     * @param shouldFlipPath      Should the path be flipped to the other side of
     *                            the field? This will
     *                            maintain a global blue alliance origin.
     * @param requirements        Subsystems required by this command, usually just
     *                            the drive subsystem
     */
    public MinoFollowPathHolonomic(
            PathPlannerPath path,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Consumer<ChassisSpeeds> outputRobotRelative,
            MinoHolonomicPathFollowerConfiguration config,
            BooleanSupplier shouldFlipPath,
            Subsystem... requirements) {
        this(
                path,
                poseSupplier,
                speedsSupplier,
                outputRobotRelative,
                config.translationConstants,
                config.rotationConstants,
                config.translationKa,
                config.maxModuleSpeed,
                config.driveBaseRadius,
                config.period,
                shouldFlipPath,
                requirements);
    }
}