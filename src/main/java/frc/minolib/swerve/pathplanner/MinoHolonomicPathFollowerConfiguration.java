package frc.minolib.swerve.pathplanner;

import com.pathplanner.lib.config.PIDConstants;

/** Configuration for the holonomic path following commands */
public class MinoHolonomicPathFollowerConfiguration {
    /** PIDConstants used for translation PID controllers */
    public final PIDConstants translationConstants;
    /** PIDConstants used for rotation PID controllers */
    public final PIDConstants rotationConstants;

    public final double translationKa;

    /** Max speed of a drive module in m/s */
    public final double maxModuleSpeed;
    /** Radius of the drive base in meters */
    public final double driveBaseRadius;
    /** Period of the robot control loop in seconds */
    public final double period;

    /**
     * Create a new holonomic path follower config
     *
     * @param translationConstants {@link com.pathplanner.lib.util.PIDConstants}
     *                             used for creating the
     *                             translation PID controllers
     * @param rotationConstants    {@link com.pathplanner.lib.util.PIDConstants}
     *                             used for creating the
     *                             rotation PID controller
     * @param maxModuleSpeed       Max speed of an individual drive module in
     *                             meters/sec
     * @param driveBaseRadius      The radius of the drive base in meters. This is
     *                             the distance from the
     *                             center of the robot to the furthest module.
     * @param replanningConfig     Path replanning configuration
     * @param period               Control loop period in seconds (Default = 0.02)
     */
    public MinoHolonomicPathFollowerConfiguration(
        PIDConstants translationConstants,
        PIDConstants rotationConstants,
        double translationKa,
        double maxModuleSpeed,
        double driveBaseRadius,
        double period
    ) {
        this.translationConstants = translationConstants;
        this.rotationConstants = rotationConstants;
        this.translationKa = translationKa;
        this.maxModuleSpeed = maxModuleSpeed;
        this.driveBaseRadius = driveBaseRadius;
        this.period = period;
    }

    /**
     * Create a new holonomic path follower config
     *
     * @param translationConstants {@link com.pathplanner.lib.util.PIDConstants}
     *                             used for creating the
     *                             translation PID controllers
     * @param rotationConstants    {@link com.pathplanner.lib.util.PIDConstants}
     *                             used for creating the
     *                             rotation PID controller
     * @param maxModuleSpeed       Max speed of an individual drive module in
     *                             meters/sec
     * @param driveBaseRadius      The radius of the drive base in meters. For
     *                             swerve drive, this is the
     *                             distance from the center of the robot to the
     *                             furthest module. For mecanum, this is the
     *                             drive base width / 2
     * @param replanningConfig     Path replanning configuration
     */
    public MinoHolonomicPathFollowerConfiguration(
        PIDConstants translationConstants,
        PIDConstants rotationConstants,
        double translationKa,
        double maxModuleSpeed,
        double driveBaseRadius
    ) {
        this(translationConstants, rotationConstants, translationKa, maxModuleSpeed, driveBaseRadius, 0.02);
    }

    /**
     * Create a new holonomic path follower config
     *
     * @param maxModuleSpeed   Max speed of an individual drive module in meters/sec
     * @param driveBaseRadius  The radius of the drive base in meters. For swerve
     *                         drive, this is the
     *                         distance from the center of the robot to the furthest
     *                         module. For mecanum, this is the
     *                         drive base width / 2
     * @param replanningConfig Path replanning configuration
     * @param period           Control loop period in seconds (Default = 0.02)
     */
    public MinoHolonomicPathFollowerConfiguration(
        double maxModuleSpeed,
        double driveBaseRadius,
        double period
    ) {
        this(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0), 0.0, maxModuleSpeed, driveBaseRadius, period);
    }

    /**
     * Create a new holonomic path follower config
     *
     * @param maxModuleSpeed   Max speed of an individual drive module in meters/sec
     * @param driveBaseRadius  The radius of the drive base in meters. For swerve
     *                         drive, this is the
     *                         distance from the center of the robot to the furthest
     *                         module. For mecanum, this is the
     *                         drive base width / 2
     * @param replanningConfig Path replanning configuration
     */
    public MinoHolonomicPathFollowerConfiguration(double maxModuleSpeed, double driveBaseRadius) {
        this(maxModuleSpeed, driveBaseRadius, 0.02);
    }
}