package frc.minolib.swerve.pathplanner;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.minolib.wpilib.PIDVController;
import frc.minolib.wpilib.ProfiledPIDVController;

import java.util.Optional;
import java.util.function.Supplier;

/** Path following controller for holonomic drive trains */
public class MinoHolonomicDriveController implements MinoPathFollowingController {
    private final PIDVController xController;
    private final PIDVController yController;
    private final ProfiledPIDVController rotationController;
    private final double maxModuleSpeed;
    private final double mpsToRps;
    private final double translationKa;

    private Translation2d translationError = new Translation2d();
    private boolean isEnabled = true;

    private static Supplier<Optional<Rotation2d>> rotationTargetOverride = null;

    private final PathPlannerPath path;
    private final PathConstraints constraints;

    /**
     * Constructs a HolonomicDriveController
     *
     * @param translationConstants PID constants for the translation PID controllers
     * @param rotationConstants    PID constants for the rotation controller
     * @param period               Period of the control loop in seconds
     * @param maxModuleSpeed       The max speed of a drive module in meters/sec
     * @param driveBaseRadius      The radius of the drive base in meters. For
     *                             swerve drive, this is the
     *                             distance from the center of the robot to the
     *                             furthest module. For mecanum, this is the
     *                             drive base width / 2
     */
    public MinoHolonomicDriveController(PathPlannerPath path, PIDConstants translationConstants, PIDConstants rotationConstants, double translationKa, double period, double maxModuleSpeed, double driveBaseRadius) {
        this.path = path;
        this.constraints = path.getGlobalConstraints();

        this.translationKa = translationKa;

        this.xController = new PIDVController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.xController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

        this.yController = new PIDVController(translationConstants.kP, translationConstants.kI, translationConstants.kD, period);
        this.yController.setIntegratorRange(-translationConstants.iZone, translationConstants.iZone);

        // Temp rate limit of 0, will be changed in calculate
        this.rotationController = new ProfiledPIDVController(
                rotationConstants.kP,
                rotationConstants.kI,
                rotationConstants.kD,
                new TrapezoidProfile.Constraints(0, 0),
                period);
        this.rotationController.setIntegratorRange(-rotationConstants.iZone, rotationConstants.iZone);
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.maxModuleSpeed = maxModuleSpeed;
        this.mpsToRps = 1.0 / driveBaseRadius;
    }

    /**
     * Constructs a HolonomicDriveController
     *
     * @param translationConstants PID constants for the translation PID controllers
     * @param rotationConstants    PID constants for the rotation controller
     * @param maxModuleSpeed       The max speed of a drive module in meters/sec
     * @param driveBaseRadius      The radius of the drive base in meters. For
     *                             swerve drive, this is the
     *                             distance from the center of the robot to the
     *                             furthest module. For mecanum, this is the
     *                             drive base width / 2
     */
    public MinoHolonomicDriveController(
        PathPlannerPath path,
        PIDConstants translationConstants,
        PIDConstants rotationConstants,
        double translationKa,
        double maxModuleSpeed,
        double driveBaseRadius
    ) {
        this(path, translationConstants, rotationConstants, translationKa, 0.02, maxModuleSpeed, driveBaseRadius);
    }

    /**
     * Enables and disables the controller for troubleshooting. When calculate() is
     * called on a
     * disabled controller, only feedforward values are returned.
     *
     * @param enabled If the controller is enabled or not
     */
    public void setEnabled(boolean enabled) {
        this.isEnabled = enabled;
    }

    /**
     * Resets the controller based on the current state of the robot
     *
     * @param currentPose   Current robot pose
     * @param currentSpeeds Current robot relative chassis speeds
     */
    @Override
    public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
        rotationController.reset(currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Calculates the next output of the path following controller
     *
     * @param currentPose The current robot pose
     * @param targetState The desired trajectory state
     * @return The next robot relative output of the path following controller
     */
    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, ChassisSpeeds currentSpeeds, PathPlannerTrajectoryState targetState) {
        // --- feedforward velocity (was targetState.velocityMps) ---
        double xFF = targetState.linearVelocity * targetState.heading.getCos();
        double yFF = targetState.linearVelocity * targetState.heading.getSin();

        // --- feedforward linear acceleration ---
        // PathPlanner 2025 exposes per-module accelerations via DriveFeedforwards.
        // We take the average across modules (same idea as using a single scalar
        // acceleration value in older APIs).
        double[] accelArray = targetState.feedforwards.accelerationsMPSSq();
        double accelMean = 0.0;
        if (accelArray != null && accelArray.length > 0) {
            double sum = 0.0;
            for (double a : accelArray) sum += a;
            accelMean = sum / accelArray.length;
        }
        double xAccelFF = accelMean * targetState.heading.getCos();
        double yAccelFF = accelMean * targetState.heading.getSin();

        // --- angular acceleration due to curvature (was curvature * v^2) ---
        // For planar motion: ω = v * curvature  => curvature = ω / v
        // therefore angularAccel = curvature * v^2 = ω * v
        double angularAccelRobotFrame = targetState.fieldSpeeds.omegaRadiansPerSecond * targetState.linearVelocity;
        xAccelFF += angularAccelRobotFrame * -targetState.heading.getSin();
        yAccelFF += angularAccelRobotFrame * targetState.heading.getCos();

        // --- translation error for diagnostics / logging ---
        this.translationError = currentPose.getTranslation().minus(targetState.pose.getTranslation());

        // If controller is disabled, just return the feedforward (field-relative -> robot)
        if (!this.isEnabled) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, 0.0, currentPose.getRotation());
        }

        // --- convert measured chassis speeds into a Translation2d (robot or field frame
        // behavior preserved from your original)
        var v = new Translation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
        v = v.rotateBy(currentPose.getRotation());

        // --- feedback terms (position controllers) ---
        double xFeedback = this.xController.calculate(
                currentPose.getX(),           // current x
                v.getX(),                    // current velocity in x
                targetState.pose.getX(),     // target x
                xFF                          // feedforward x
        );

        double yFeedback = this.yController.calculate(
                currentPose.getY(),           // current y
                v.getY(),                    // current velocity in y
                targetState.pose.getY(),     // target y
                yFF                          // feedforward y
        );

        // --- angular velocity constraint and trapezoid profile ---
        // NOTE: PathConstraints accessors in 2025 are named *.maxAngularVelocityRadPerSec()
        // and *.maxAngularAccelerationRadPerSecSq(). If targetState.constraints is not
        // accessible in your package (it's protected on the state), see the caveats below.
        double angVelConstraint = Double.POSITIVE_INFINITY;
        try {
            angVelConstraint = constraints.maxAngularVelocityRadPerSec();
        } catch (Throwable t) {
            // fallback if constraints are not accessible (protected) — leave infinite
            angVelConstraint = Double.POSITIVE_INFINITY;
        }
        double maxAngVel = angVelConstraint;

        if (Double.isFinite(maxAngVel)) {
            double maxAngVelModule = Math.max(0.0, maxModuleSpeed - targetState.linearVelocity) * mpsToRps;
            maxAngVel = Math.min(angVelConstraint, maxAngVelModule);
        }

        var rotationConstraints = new TrapezoidProfile.Constraints(
                maxAngVel,
                // use PathConstraints accessor for angular acc if available
                (Double.isFinite(maxAngVel) && constraints != null) ? constraints.maxAngularAccelerationRadPerSecSq() : targetState.fieldSpeeds.omegaRadiansPerSecond // fallback small value
        );

        // --- rotation setpoint / feedforward ---
        // In older API you used targetState.targetHolonomicRotation and targetState.holonomicAngularVelocityRps.
        // PathPlanner 2025 gives you fieldSpeeds.omegaRadiansPerSecond (the desired angular rate).
        Rotation2d targetRotation = (rotationTargetOverride != null) ? rotationTargetOverride.get().orElse(targetState.heading) : targetState.heading; // fallback to heading

        // Create a trapezoid state: use desired angular velocity from the trajectory state if present.
        double targetAngularVel = targetState.fieldSpeeds.omegaRadiansPerSecond;
        double rotationFeedback = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                currentSpeeds.omegaRadiansPerSecond,
                new TrapezoidProfile.State(targetRotation.getRadians(), targetAngularVel),
                rotationConstraints
        );

        double rotationFF = rotationController.getSetpoint().velocity;

        // --- final robot-relative chassis speeds ---
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback + xAccelFF * translationKa,
                yFF + yFeedback + yAccelFF * translationKa,
                rotationFF + rotationFeedback,
                currentPose.getRotation()
        );
    }

    /**
     * Get the current positional error between the robot's actual and target
     * positions
     *
     * @return Positional error, in meters
     */
    @Override
    public double getPositionalError() {
        return translationError.getNorm();
    }

    /**
     * Is this controller for holonomic drivetrains? Used to handle some differences
     * in functionality
     * in the path following command.
     *
     * @return True if this controller is for a holonomic drive train
     */
    @Override
    public boolean isHolonomic() {
        return true;
    }

    /**
     * Set a supplier that will be used to override the rotation target when path
     * following.
     *
     * <p>
     * This function should return an empty optional to use the rotation targets in
     * the path
     *
     * @param rotationTargetOverride Supplier to override rotation targets
     */
    public static void setRotationTargetOverride(Supplier<Optional<Rotation2d>> rotationTargetOverride) {
        MinoHolonomicDriveController.rotationTargetOverride = rotationTargetOverride;
    }
}