package frc.minolib.swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class MinoSwerveTeleopControl {
    private final SlewRateLimiter xVelRateLimiter;
    private final SlewRateLimiter yVelRateLimiter;
    private final SlewRateLimiter angularRateLimiter;
    private final double stickDeadband;
    private final double maxDriveSpeed;
    private final double maxAngularVelocity;
    private final boolean applySoftPlus;

    /**
     * Helper to compute drive velocities from joystick values.
     *
     * @param linearSlewRate m/s/s
     * @param angularSlewRate rad/s/s
     * @param stickDeadband joystick deadband as a percentage (0 to 1.0)
     * @param maxDriveSpeed m/s/s
     * @param maxAngularVelocity rad/s/s
     * @param applySoftPlus whether or not to apply SoftPlus to the deadbanded joystick
     */

    public MinoSwerveTeleopControl(final double linearSlewRate, final double angularSlewRate, final double stickDeadband, final double maxDriveSpeed, final double maxAngularVelocity, final boolean applySoftPlus) {
        xVelRateLimiter = new SlewRateLimiter(linearSlewRate);
        yVelRateLimiter = new SlewRateLimiter(linearSlewRate);
        angularRateLimiter = new SlewRateLimiter(angularSlewRate);
        this.stickDeadband = stickDeadband;
        this.maxDriveSpeed = maxDriveSpeed;
        this.maxAngularVelocity = maxAngularVelocity;
        this.applySoftPlus = applySoftPlus;
    }

    public class DriveVelocities {
        public final double xVel; // m/s
        public final double yVel; // m/s
        public final double thetaVel; // rad/s

        public DriveVelocities(final double xVel, final double yVel, final double thetaVel) {
        this.xVel = xVel;
        this.yVel = yVel;
        this.thetaVel = thetaVel;
        }
    }

    /**
     * Compute the translation and rotation velocities given joystick axis inputs.
     *
     * @param xAxis Joystick axis that corresponds to the forward axis of the robot.
     * @param yAxis Joystick axis that corresponds to the left axis of the robot.
     * @param rAxis Joystick axis that corresponds to the CCW rotation of the robot.
     */
    public DriveVelocities getDriveVelocitiesFromJoysticks(double xAxis, double yAxis, double rAxis) {

        /* Deadbands */
        final var deadbandedXY = applyDeadbandOnXYAndScale(xAxis, yAxis, stickDeadband, applySoftPlus);
        xAxis = deadbandedXY.getFirst();
        yAxis = deadbandedXY.getSecond();
        rAxis = applyDeadband(rAxis, stickDeadband);

        if (applySoftPlus) {
            // xy scaling handled in applyDeadbandOnXYAndScale
            rAxis = applySoftPlus(rAxis);
        }

        // Convert joystick axes to velocities.
        final double xVel = xAxis * maxDriveSpeed;
        final double yVel = yAxis * maxDriveSpeed;
        final double angularVel = rAxis * maxAngularVelocity;

        // Apply slew rates on velocities.
        return new DriveVelocities(xVelRateLimiter.calculate(xVel), yVelRateLimiter.calculate(yVel), angularRateLimiter.calculate(angularVel));
    }

    /**
     * Apply deadband in joystick polar coordinates instead of on the raw joystick values. Scale the
     * polar value.
     */
    private static Pair<Double, Double> applyDeadbandOnXYAndScale(
        final double x, final double y, final double deadband, final boolean applySoftPlus) {
        final double polarR = Math.hypot(x, y);
        if (polarR == 0.0) {
            return new Pair<>(0.0, 0.0);
        }

        final double deadbandedR = applyDeadband(polarR, deadband);
        final double scaledCoefficient = (applySoftPlus ? applySoftPlus(deadbandedR) : deadbandedR) / polarR;
        return new Pair<>(x * scaledCoefficient, y * scaledCoefficient);
    }

    /**
     * Apply deadband. Ramps linearly from 0 to 1 from (deadband, 1) and 0 to -1 from (-deadband, -1).
     */
    public static double applyDeadband(final double value, final double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        } else {
            final double signedOne = Math.signum(value);
            return ((value - signedOne) / (1.0 - deadband)) + signedOne;
        }
    }

    private static double applySoftPlus(final double x) {
        // Values are from https://www.desmos.com/calculator/kskrpihcev
        // TODO: Use more intuitive parameters and solve for these values exactly so the curve passes
        // through (0, 0) and (1, 1).
        final double L = 20.0;
        final double K = -2.0;
        final double T = 0.8927;
        final double U = 0.0025;

        final double exponent = (L * Math.abs(x) + K) / T;
        final double softPlus = Math.log(1 + T * Math.exp(exponent)) / L - U;
        return Math.signum(x) * softPlus;
    }
}