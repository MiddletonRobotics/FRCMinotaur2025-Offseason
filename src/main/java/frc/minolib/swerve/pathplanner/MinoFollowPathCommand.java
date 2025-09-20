package frc.minolib.swerve.pathplanner;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Base command for following a path */
public class MinoFollowPathCommand extends Command {
    private final Timer timer = new Timer();
    private final PathPlannerPath originalPath;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final Consumer<ChassisSpeeds> output;
    private final MinoPathFollowingController controller;
    private final BooleanSupplier shouldFlipPath;

    // NEW: EventScheduler from PathPlanner 2025
    private final EventScheduler eventScheduler = new EventScheduler();

    private PathPlannerPath path;
    private PathPlannerTrajectory generatedTrajectory;

    public MinoFollowPathCommand(
        PathPlannerPath path,
        Supplier<Pose2d> poseSupplier,
        Supplier<ChassisSpeeds> speedsSupplier,
        Consumer<ChassisSpeeds> outputRobotRelative,
        MinoPathFollowingController controller,
        BooleanSupplier shouldFlipPath,
        Subsystem... requirements
    ) {
        this.originalPath = path;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.output = outputRobotRelative;
        this.controller = controller;
        this.shouldFlipPath = shouldFlipPath;

        // Add drive requirements
        Set<Subsystem> driveRequirements = Set.of(requirements);
        addRequirements(driveRequirements);

        // IMPORTANT: add all event requirements for this path at construction time
        // so the Command framework knows about them. This is the recommended pattern.
        addRequirements(EventScheduler.getSchedulerRequirements(originalPath));
    }

    @Override
    public void initialize() {
        if (shouldFlipPath.getAsBoolean() && !originalPath.preventFlipping) {
            path = originalPath.flipPath();
        } else {
            path = originalPath;
        }

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        controller.reset(currentPose, currentSpeeds);

        // RobotConfig can be loaded from GUI settings or passed in:
        RobotConfig robotConfig;
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // fallback or rethrow depending on your system
            throw new RuntimeException("Failed to load RobotConfig for PathPlanner", e);
        }

        generatedTrajectory = new PathPlannerTrajectory(path, currentSpeeds, currentPose.getRotation(), robotConfig);

        // Initialize the EventScheduler with the generated trajectory
        eventScheduler.initialize(generatedTrajectory);

        PathPlannerLogging.logActivePath(path);
        PPLibTelemetry.setCurrentPath(path);

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double elapsed = timer.get();
        var targetState = generatedTrajectory.sample(elapsed);

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        // Your controller produces robot-relative chassis speeds
        ChassisSpeeds targetSpeeds = controller.calculateRobotRelativeSpeeds(currentPose, currentSpeeds, targetState);

        // telemetry...
        PPLibTelemetry.setCurrentPose(currentPose);
        PathPlannerLogging.logCurrentPose(currentPose);

        output.accept(targetSpeeds);

        // Tell the scheduler to run events at the current trajectory time
        eventScheduler.execute(elapsed);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(generatedTrajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        // stop the drivetrain if we finished naturally and the path ends stopped
        if (!interrupted && generatedTrajectory.getEndState().linearVelocity < 0.1) {
            output.accept(new ChassisSpeeds());
        }

        // Clean up any running event commands
        eventScheduler.end();

        PathPlannerLogging.logActivePath(null);
    }
}