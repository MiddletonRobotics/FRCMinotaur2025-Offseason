package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.minolib.hardware.CANDeviceID;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.PIDConfiguration;
import frc.minolib.swerve.MinoSwerveController;
import frc.robot.Robot;

public class DrivetrainConstants {
    public static final double maxDriveSpeed = 4.5; // m/s
    public static final double maxDriveAcceleration = 8.0; // m/s/s
    public static final double maxAngularVelocity = Math.PI * 2.0; // rad/s
    public static final double maxAngularAcceleration = Math.PI * 4.0; // rad/s/s
    public static final double trackWidth = Units.inchesToMeters(24.00);
    public static final double wheelBase = Units.inchesToMeters(24.00);
    public static final double wheelDiameter = Units.inchesToMeters(3.91);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final MechanismRatio driveRatio = new MechanismRatio(1.0, (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0), wheelCircumference);
    public static final MechanismRatio steeringRatio = new MechanismRatio(1.0, 18.75);
    public static final double steerDriveCouplingRatio = 50.0 / 16.0;
    public static final PIDConfiguration driveOpenLoopPIDConfig = new PIDConfiguration(0.0, 0.0, 0.0, 0.1, 0.125, 0.0, 0.0);
    public static final PIDConfiguration driveClosedLoopPIDConfig = new PIDConfiguration(0.3, 0.0, 0.0, 0.1, 0.125, 0.005, 0.0);
    public static final PIDConfiguration steeringPIDConfig = Robot.isSimulation() ? new PIDConfiguration(1.5, 0.0, 0.0) : new PIDConfiguration(5.0, 0.0, 0.01);

    /* Swerve Teleop Slew Rate Limits */
    public static final double linearSlewRate = 60.0; // m/s/s
    public static final double angularSlewRate = 120.0; // rad/s/s
    public static final double stickDeadband = 0.05;

    /* Module Slew Rate Limits */
    public static final double maxModuleAcceleration = 60.0; // m/s/s
    public static final double maxModuleSteeringRate = 4.0 * Math.PI; // rad/s/s

    /* Allowable scrub */
    public static final double autoScrubLimit = 0.25; // m/s
    public static final double teleopScrubLimit = 0.25; // m/s

    public static final MinoSwerveController driveController = new MinoSwerveController(
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(5.0, 0.0, 0.0),
        new PIDController(4.0, 0.0, 0.0)
    );

    public static final class IMU {
        public static final CANDeviceID pigeonID = new CANDeviceID(14, GlobalConstants.kCanivoreName);
        public static final double gyroTrimZ = -0.05418;
    }

    /* Front Left Module - Module 0 */
    public static final class FrontLeft {
        public static final CANDeviceID driveMotorID = new CANDeviceID(2, GlobalConstants.kCanivoreName);
        public static final CANDeviceID steeringMotorID = new CANDeviceID(3, GlobalConstants.kCanivoreName);
        public static final CANDeviceID canCoderID = new CANDeviceID(10, GlobalConstants.kCanivoreName);
        public static final Translation2d modulePosition = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final double absEncoderOffsetRad = 0.195557 * 2.0 * Math.PI;
    }

    /* Rear Left Module - Module 1 */
    public static final class RearLeft {
        public static final CANDeviceID driveMotorID = new CANDeviceID(6, GlobalConstants.kCanivoreName);
        public static final CANDeviceID steeringMotorID = new CANDeviceID(7, GlobalConstants.kCanivoreName);
        public static final CANDeviceID canCoderID = new CANDeviceID(12, GlobalConstants.kCanivoreName);
        public static final Translation2d modulePosition = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final double absEncoderOffsetRad = 0.373535 * 2.0 * Math.PI;
    }

    /* Rear Right Module - Module 2 */
    public static final class RearRight {
        public static final CANDeviceID driveMotorID = new CANDeviceID(8, GlobalConstants.kCanivoreName);
        public static final CANDeviceID steeringMotorID = new CANDeviceID(9, GlobalConstants.kCanivoreName);
        public static final CANDeviceID canCoderID = new CANDeviceID(13, GlobalConstants.kCanivoreName);
        public static final Translation2d modulePosition = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);
        public static final double absEncoderOffsetRad = 0.510742 * 2.0 * Math.PI;
    }

    /* Front Right Module - Module 3 */
    public static final class FrontRight {
        public static final CANDeviceID driveMotorID = new CANDeviceID(4, GlobalConstants.kCanivoreName);
        public static final CANDeviceID steeringMotorID = new CANDeviceID(5, GlobalConstants.kCanivoreName);
        public static final CANDeviceID canCoderID = new CANDeviceID(11, GlobalConstants.kCanivoreName);
        public static final Translation2d modulePosition = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final double absEncoderOffsetRad = 0.45652 * 2.0 * Math.PI;
    }
}
