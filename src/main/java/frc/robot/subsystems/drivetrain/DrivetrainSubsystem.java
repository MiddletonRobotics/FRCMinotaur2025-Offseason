package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.minolib.RobotConfiguration;
import frc.minolib.advantagekit.LoggedTracer;
import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.localization.MinoPoseEstimator;
import frc.minolib.localization.MinoRobotOdometry;
import frc.minolib.wpilib.SysIDRoutineChooser;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.DrivetrainConstants.SysIDCharacterizationMode;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.utilities.Conversions;
import frc.robot.utilities.Field2d;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * This subsystem models the robot's drivetrain mechanism. It consists of a four swerve modules in
 * the MK4 family, each with two TalonFX motors and a CANcoder. It also consists of a Pigeon which
 * is used to measure the robot's rotation.
 */
public class DrivetrainSubsystem extends SubsystemBase implements MinoPoseEstimator {

  private final DrivetrainIO io;
  private final DrivetrainIO.DrivetrainIOInputsCollection inputs = new DrivetrainIO.DrivetrainIOInputsCollection();

  /*
  * If TUNING is set to true in Constants.java, the following tunables will be available in AdvantageScope. This enables efficient tuning of PID coefficients without restarting the code.
  */

  private final LoggedTunableNumber autoDriveKp = new LoggedTunableNumber("AutoDrive/DriveKp", RobotConfiguration.getInstance().getAutoDriveKP());
  private final LoggedTunableNumber autoDriveKi = new LoggedTunableNumber("AutoDrive/DriveKi", RobotConfiguration.getInstance().getAutoDriveKI());
  private final LoggedTunableNumber autoDriveKd = new LoggedTunableNumber("AutoDrive/DriveKd", RobotConfiguration.getInstance().getAutoDriveKD());
  private final LoggedTunableNumber autoTurnKp = new LoggedTunableNumber("AutoDrive/TurnKp", RobotConfiguration.getInstance().getAutoTurnKP());
  private final LoggedTunableNumber autoTurnKi = new LoggedTunableNumber("AutoDrive/TurnKi", RobotConfiguration.getInstance().getAutoTurnKI());
  private final LoggedTunableNumber autoTurnKd = new LoggedTunableNumber("AutoDrive/TurnKd", RobotConfiguration.getInstance().getAutoTurnKD());

  private final PIDController autoXController = new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoYController = new PIDController(autoDriveKp.get(), autoDriveKi.get(), autoDriveKd.get());
  private final PIDController autoThetaController = new PIDController(autoTurnKp.get(), autoTurnKi.get(), autoTurnKd.get());

  private boolean isFieldRelative = true;
  private boolean isTranslationSlowMode = false;
  private boolean isRotationSlowMode = false;

  // Set to true upon construction to trigger disabling break mode shortly after the code starts
  private boolean brakeMode = true;
  private Timer brakeModeTimer = new Timer();
  private static final double BREAK_MODE_DELAY_SEC = 10.0;

  private boolean isMoveToPoseEnabled = true;

  private SlewRateLimiter xFilter = new SlewRateLimiter(8);
  private SlewRateLimiter yFilter = new SlewRateLimiter(8);
  // private SlewRateLimiter thetaFilter = new SlewRateLimiter(Units.degreesToRadians(360));

  private boolean accelerationLimiting = false;
  private boolean driveToPoseCanceled = false;

  private Alert noPoseAlert = new Alert("Attempted to reset pose from vision, but no pose was found.", AlertType.kWarning);
  private Alert pathFileMissingAlert = new Alert("Could not find the specified path file.", AlertType.kError);

  private DriverStation.Alliance alliance = Field2d.getInstance().getAlliance();

  private final MinoRobotOdometry odometry;
  private Pose2d prevRobotPose = new Pose2d();
  private int teleportedCount = 0;
  private int constrainPoseToFieldCount = 0;

  private Pose2d customPose = new Pose2d();

  private double[] initialDistance = {0.0, 0.0, 0.0, 0.0};
  private boolean isRotationOverrideEnabled = false;

  private SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition(),
    new SwerveModulePosition()
  };

  /**
   * SysId routine for characterizing translation. This is used to find FF/PID gains for the drive
   * motors. This should be used when the drive motors are using voltage control.
   */

  private final SysIdRoutine sysIdRoutineTranslationVolts = new SysIdRoutine(
    new SysIdRoutine.Config(null, Volts.of(4), null, state -> SignalLogger.writeString("SysIdTranslationVolts_State", state.toString())),
    new SysIdRoutine.Mechanism(output -> applySysIdCharacterization(SysIDCharacterizationMode.TRANSLATION_VOLTS, output.in(Volts)), null, this)
  );

  /**
   * SysId routine for characterizing translation. This is used to find FF/PID gains for the drive
   * motors. This should be used when the drive motors are using TorqueCurrentFOC control.
   */

  private final SysIdRoutine sysIdRoutineTranslationCurrent = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(10).per(Second), Volts.of(20), Seconds.of(5), state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
    new SysIdRoutine.Mechanism(output -> applySysIdCharacterization(SysIDCharacterizationMode.TRANSLATION_CURRENT, output.in(Volts)), null, this)
  );

  /**
   * SysId routine for characterizing steer. This is used to find FF/PID gains for the steer motors.
   * This should be used when the steer motors are using voltage control.
   */

  private final SysIdRoutine sysIdRoutineSteerVolts = new SysIdRoutine(
    new SysIdRoutine.Config(null, Volts.of(7), null, state -> SignalLogger.writeString("SysIdSteerVolts_State", state.toString())),
    new SysIdRoutine.Mechanism(volts -> applySysIdCharacterization(SysIDCharacterizationMode.STEER_VOLTS, volts.in(Volts)), null, this)
  );

  /**
   * SysId routine for characterizing steer. This is used to find FF/PID gains for the steer motors.
   * This should be used when the steer motors are using TorqueCurrentFOC control.
   */

  private final SysIdRoutine sysIdRoutineSteerCurrent = new SysIdRoutine(
    new SysIdRoutine.Config(Volts.of(10).per(Second), Volts.of(20), Seconds.of(5), state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
    new SysIdRoutine.Mechanism(output -> applySysIdCharacterization(SysIDCharacterizationMode.STEER_CURRENT,output.in(Volts)), null, this)
  );

  /*
  * SysId routine for characterizing rotation.
  * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
  * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
  */

  private final SysIdRoutine sysIdRoutineRotation =
    new SysIdRoutine(new SysIdRoutine.Config(Volts.of(Math.PI / 6).per(Second), Volts.of(Math.PI), null, state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
    new SysIdRoutine.Mechanism( output -> {
      /* output is actually radians per second, but SysId only supports "volts" */
      applySysIdCharacterization(SysIDCharacterizationMode.ROTATION_VOLTS, output.in(Volts));
      /* also log the requested output for SysId */
      SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    }, null, this)
  );

  /**
   * Creates a new Drivetrain subsystem.
   * @param io the abstracted interface for the drivetrain
   */

  public DrivetrainSubsystem(DrivetrainIO io) {
    this.io = io;
    this.autoThetaController.enableContinuousInput(-Math.PI, Math.PI);

    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      this::applyRobotSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      new PPHolonomicDriveController( 
        new com.pathplanner.lib.config.PIDConstants(
          RobotConfiguration.getInstance().getAutoDriveKP(),
          RobotConfiguration.getInstance().getAutoDriveKI(),
          RobotConfiguration.getInstance().getAutoDriveKD()
        ), // Translation PID constants
        new PIDConstants(
          RobotConfiguration.getInstance().getAutoTurnKP(),
          RobotConfiguration.getInstance().getAutoTurnKI(),
          RobotConfiguration.getInstance().getAutoTurnKD()
        ) // Rotation PID constants
      ),
      RobotConfiguration.getInstance().getPathPlannerRobotConfig(),
      this::shouldFlipAutoPath,
      this // Reference to this subsystem to set requirements
    );

    this.odometry = MinoRobotOdometry.getInstance();
    this.odometry.setposeEstimator(this);

    this.xFilter.reset(0.0);
    this.yFilter.reset(0.0);
    // this.thetaFilter.reset(0.0);

    SysIDRoutineChooser.getInstance().addOption("Translation Volts", sysIdRoutineTranslationVolts);
    SysIDRoutineChooser.getInstance().addOption("Translation Current", sysIdRoutineTranslationCurrent);
    SysIDRoutineChooser.getInstance().addOption("Steer Volts", sysIdRoutineSteerVolts);
    SysIDRoutineChooser.getInstance().addOption("Steer Current", sysIdRoutineSteerCurrent);
    SysIDRoutineChooser.getInstance().addOption("Rotation Volts", sysIdRoutineRotation);
}

  /**
   * Returns the robot-relative speeds of the robot.
   * @return the robot-relative speeds of the robot
   */

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return this.inputs.drivetrain.measuredChassisSpeeds;
  }

  /**
   * Applies the specified robot-relative speeds to the drivetrain along with the specified feed
   * forward forces.
   *
   * @param chassisSpeeds the robot-relative speeds of the robot
   * @param feedforwards the feed forward forces to apply
   */

  public void applyRobotSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards feedforwards) {
    this.io.applyRobotSpeeds(chassisSpeeds, feedforwards.robotRelativeForcesX(), feedforwards.robotRelativeForcesY(), false);
  }

  /**
   * Zeroes the gyroscope. This sets the current rotation of the robot to zero degrees. This method
   * is intended to be invoked only when the alignment between the robot's rotation and the gyro is
   * sufficiently different to make field-relative driving difficult. The robot needs to be
   * positioned facing away from the driver, ideally aligned to a field wall before this method is
   * invoked.
   */

  public void zeroGyroscope() {
    setGyroOffset(0.0);
  }

  /**
   * Returns the rotation of the robot. Zero degrees is facing away from the driver station; CCW is
   * positive.
   *
   * @return the rotation of the robot
   */

  public Rotation2d getRotation() {
    return this.odometry.getEstimatedPose().getRotation();
  }

  /**
   * Returns the raw heading of the drivetrain as reported by the gyro in degrees. Usually, the
   * getRotation method should be invoked instead.
   *
   * @return the raw heading of the drivetrain as reported by the gyro in degrees
   */

  public double getYaw() {
    return this.inputs.drivetrain.rawHeadingDeg;
  }

  /**
   * Sets the rotation of the robot to the specified value. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). Usually, the
   * resetPose method is used instead. Zero degrees is facing away from the driver station; CCW is
   * positive.
   *
   * @param expectedYaw the rotation of the robot (in degrees)
   */

  public void setGyroOffset(double expectedYaw) {
    this.resetPose(new Pose2d(this.odometry.getEstimatedPose().getTranslation(), Rotation2d.fromDegrees(expectedYaw)));
  }

  /**
   * Returns the pose of the robot (e.g., x and y position of the robot on the field and the robot's
   * rotation). The origin of the field is always the blue origin (i.e., the positive x-axis points
   * away from the blue alliance wall). Zero degrees is aligned to the positive x axis and increases
   * in the CCW direction.
   *
   * @return the pose of the robot
   */

  public Pose2d getPose() {
    return this.odometry.getEstimatedPose();
  }

  /**
   * Sets the odometry of the robot to the specified pose. This method should only be invoked when
   * the rotation of the robot is known (e.g., at the start of an autonomous path). The origin of
   * the field is always the blue origin (i.e., the positive x-axis points away from the blue
   * alliance wall). Zero degrees is aligned to the positive x axis and increases in the CCW
   * direction.
   *
   * @param pose the specified pose to which is set the odometry
   */

  public void resetPose(Pose2d pose) {
    this.odometry.resetPose(Rotation2d.fromDegrees(this.inputs.drivetrain.rawHeadingDeg), this.modulePositions, pose);
    this.prevRobotPose = pose;
  }

  /**
   * Sets the odometry of the robot based on the supplied pose (e.g., from the vision subsystem).
   * The robot can be positioned in front of an AprilTag and this method can be invoked to reset the
   * robot's pose based on tag.
   *
   * @param poseSupplier the supplier of the pose to which set the robot's odometry
   */

  public void resetPoseToVision(Supplier<Pose3d> poseSupplier) {
    Pose3d pose = poseSupplier.get();
    if (pose != null) {
      noPoseAlert.set(false);
      this.resetPose(pose.toPose2d());
    } else {
      noPoseAlert.set(true);
    }
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x, y, and
   * rotational directions. The velocities may be specified from either the robot's frame of
   * reference or the field's frame of reference. In the robot's frame of reference, the positive x
   * direction is forward; the positive y direction, left; position rotation, CCW. In the field
   * frame of reference, the positive x direction is away from the blue alliance wall and the
   * positive y direction is to a blue alliance driver's left. This method accounts for the fact
   * that the origin of the field is always the corner to the right of the blue alliance driver
   * station. A positive rotational velocity always rotates the robot in the CCW direction.
   *
   * <p>If the translation or rotation slow mode features are enabled, the corresponding velocities
   * will be scaled to enable finer control.
   *
   * <p>If the drive mode is X, the robot will ignore the specified velocities and turn the swerve
   * modules into the x-stance orientation.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param rotationalVelocity the desired rotational velocity (rad/s)
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   * @param isFieldRelative true to for field-relative motion; false, for robot-relative
   */

  public void drive(double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop, boolean isFieldRelative) {
    // get the slow-mode multiplier from the config
    double slowModeMultiplier = RobotConfiguration.getInstance().getRobotSlowModeMultiplier();

    // log velocity before and after filter
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/requestedXVelocity", xVelocity);
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/requestedYVelocity", yVelocity);
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/requestedRotationalVelocity", rotationalVelocity);

    // should we give it the actual current velocity or the desired velocity?
    // always calculate whenever we are driving so that we maintain a history of recent values
    // find the other method that the autobuilder uses to drive

    this.xFilter.calculate(xVelocity);
    this.yFilter.calculate(yVelocity);
    // this.thetaFilter.calculate(rotationalVelocity);

    // log velocity after filter
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/filteredXVelocity", this.xFilter.lastValue());
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/filteredYVelocity", this.yFilter.lastValue());
    // Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/filteredRotationalVelocity", this.thetaFilter.lastValue());

    if (accelerationLimiting) {
      xVelocity = this.xFilter.lastValue();
      yVelocity = this.yFilter.lastValue();
      // rotationalVelocity = this.thetaFilter.lastValue();
    }

    // if translation or rotation is in slow mode, multiply the x and y velocities by the slow-mode multiplier
    if (isTranslationSlowMode) {
      xVelocity *= slowModeMultiplier;
      yVelocity *= slowModeMultiplier;
    }

    if (GlobalConstants.kDemoMode) {
      double velocity = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));
      if (velocity > DrivetrainConstants.kDemoModeMaximumVelocity) {
          double scale = DrivetrainConstants.kDemoModeMaximumVelocity / velocity;
          xVelocity *= scale;
          yVelocity *= scale;
      }
    }

    // if rotation is in slow mode, multiply the rotational velocity by the slow-mode multiplier
    if (isRotationSlowMode) {
      rotationalVelocity *= slowModeMultiplier;
    }

    if (isFieldRelative) {
      // the origin of the field is always the corner to the right of the blue alliance driver
      // station. As a result, "forward" from a field-relative perspective when on the red
      // alliance, is in the negative x direction. Similarly, "left" from a field-relative
      // perspective when on the red alliance is in the negative y direction.
      int allianceMultiplier = this.alliance == Alliance.Blue ? 1 : -1;
      this.io.driveFieldRelative(allianceMultiplier * xVelocity, allianceMultiplier * yVelocity, rotationalVelocity, isOpenLoop);
    } else {
      this.io.driveRobotRelative(xVelocity, yVelocity, rotationalVelocity, isOpenLoop);
    }
  }

  /**
   * Controls the drivetrain to move the robot with the desired velocities in the x and y
   * directions, while keeping the robot aligned to the specified target rotation. The velocities
   * must be specified from the field's frame of reference as field relative mode is assumed. In the
   * field frame of reference, the origin of the field is always the blue origin (i.e., the positive
   * x-axis points away from the blue alliance wall). Zero degrees is aligned to the positive x axis
   * and increases in the CCW direction.
   *
   * <p>If the translation slow mode feature is enabled, the corresponding velocities will be scaled
   * to enable finer control.
   *
   * @param xVelocity the desired velocity in the x direction (m/s)
   * @param yVelocity the desired velocity in the y direction (m/s)
   * @param targetDirection the desired direction of the robot's orientation. Zero degrees is aligned to the positive x axis and increases in the CCW direction.
   * @param isOpenLoop true for open-loop control; false for closed-loop control
   */

  public void driveFacingAngle(double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {
    // get the slow-mode multiplier from the config
    double slowModeMultiplier = RobotConfiguration.getInstance().getRobotSlowModeMultiplier();

    // if translation or rotation is in slow mode, multiply the x and y velocities by the
    // slow-mode multiplier
    if (isTranslationSlowMode) {
      xVelocity *= slowModeMultiplier;
      yVelocity *= slowModeMultiplier;
    }

    if (GlobalConstants.kDemoMode) {
      double velocity = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));
      if (velocity > DrivetrainConstants.kDemoModeMaximumVelocity) {
          double scale = DrivetrainConstants.kDemoModeMaximumVelocity / velocity;
          xVelocity *= scale;
          yVelocity *= scale;
      }
    }

    int allianceMultiplier = this.alliance == Alliance.Blue ? 1 : -1;
    this.io.driveFieldRelativeFacingAngle(xVelocity * allianceMultiplier, yVelocity * allianceMultiplier, targetDirection, isOpenLoop);

    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/DriveFacingAngle/targetDirection", targetDirection);
  }

  /**
   * Stops the motion of the robot. Since the motors are in brake mode, the robot will stop soon
   * after this method is invoked.
   */
  public void stop() {
    this.io.driveRobotRelative(0.0, 0.0, 0.0, false);
  }

  /**
   * Puts the drivetrain into the x-stance orientation. In this orientation the wheels are aligned
   * to make an 'X'. This prevents the robot from rolling on an inclined surface and makes it more
   * difficult for other robots to push the robot, which is useful when shooting.
   */
  public void holdXstance() {
    this.io.holdXStance();
  }

  /**
   * This method is invoked each iteration of the scheduler. The primarily responsibility is to
   * update the drivetrain inputs from the hardware-specific layer. In addition, the odometry of the
   * robot, brake mode, and controllers are updated.
   */
  @SuppressWarnings("unused")
  @Override
  public void periodic() {
    this.io.updateInputs(this.inputs);
    Logger.processInputs(DrivetrainConstants.kSubsystemName, this.inputs.drivetrain);
    Logger.processInputs(DrivetrainConstants.kSubsystemName + "/FL", this.inputs.swerve[0]);
    Logger.processInputs(DrivetrainConstants.kSubsystemName + "/FR", this.inputs.swerve[1]);
    Logger.processInputs(DrivetrainConstants.kSubsystemName + "/BL", this.inputs.swerve[2]);
    Logger.processInputs(DrivetrainConstants.kSubsystemName + "/BR", this.inputs.swerve[3]);

    // update odometry
    for (int i = 0; i < inputs.drivetrain.odometryTimestamps.length; i++) {
      for (int moduleIndex = 0; moduleIndex < this.modulePositions.length; moduleIndex++) {
        this.modulePositions[moduleIndex].distanceMeters = inputs.swerve[moduleIndex].odometryDrivePositionsMeters[i];
        this.modulePositions[moduleIndex].angle = inputs.swerve[moduleIndex].odometryTurnPositions[i];
      }

      this.odometry.updateWithTime(inputs.drivetrain.odometryTimestamps[i], inputs.drivetrain.odometryYawPositions[i], modulePositions);
    }

    // custom pose vs default pose
    Pose2d pose = this.odometry.getEstimatedPose();
    this.customPose = this.inputs.drivetrain.customPose;

    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/Pose", pose);
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/CustomPose", this.customPose);
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/FRPose", pose.transformBy(new Transform2d(RobotConfiguration.getInstance().getFrontRightCornerPosition(), new Rotation2d())));

    // check for teleportation
    if (DrivetrainConstants.kEnableTeleportDetection && pose.minus(prevRobotPose).getTranslation().getNorm() > DrivetrainConstants.kTeleportDetectionThresholdMeters) {
      this.resetPose(prevRobotPose);
      this.teleportedCount++;
      Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/TeleportedPose", pose);
      Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/TeleportCount", this.teleportedCount);
    } else {
      this.prevRobotPose = pose;
    }

    // check for position outside the field due to slipping
    if (pose.getX() < 0) {
      this.resetPose(new Pose2d(0, pose.getY(), pose.getRotation()));
      this.constrainPoseToFieldCount++;
    } else if (pose.getX() > FieldConstants.fieldLength) {
      this.resetPose(new Pose2d(FieldConstants.fieldLength, pose.getY(), pose.getRotation()));
      this.constrainPoseToFieldCount++;
    }

    if (pose.getY() < 0) {
      this.resetPose(new Pose2d(pose.getX(), 0, pose.getRotation()));
      this.constrainPoseToFieldCount++;
    } else if (pose.getY() > FieldConstants.fieldWidth) {
      this.resetPose(new Pose2d(pose.getX(), FieldConstants.fieldWidth, pose.getRotation()));
      this.constrainPoseToFieldCount++;
    }

    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/ConstrainPoseToFieldCount", this.constrainPoseToFieldCount);
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/FieldRelative", this.getFieldRelative());
    Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/Speed", Math.hypot(inputs.drivetrain.measuredChassisSpeeds.vxMetersPerSecond, inputs.drivetrain.measuredChassisSpeeds.vyMetersPerSecond));

    // update the brake mode based on the robot's velocity and state (enabled/disabled)
    updateBrakeMode();

    // update tunables
    LoggedTunableNumber.ifChanged(
      hashCode(),
      pid -> {
        autoXController.setPID(pid[0], pid[1], pid[2]);
        autoYController.setPID(pid[0], pid[1], pid[2]);
      },
      autoDriveKp,
      autoDriveKi,
      autoDriveKd
    );

    LoggedTunableNumber.ifChanged(
      hashCode(),
      pid -> autoThetaController.setPID(pid[0], pid[1], pid[2]),
      autoTurnKp,
      autoTurnKi,
      autoTurnKd
    );

    // Record cycle time
    LoggedTracer.record("Drivetrain");
  }

  /**
   * Returns true if field relative mode is enabled
   *
   * @return true if field relative mode is enabled
   */

  public boolean getFieldRelative() {
    return isFieldRelative;
  }

  /**
   * Enables field-relative mode. When enabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the field.
   */

  public void enableFieldRelative() {
    this.isFieldRelative = true;
  }

  /**
   * Disables field-relative mode. When disabled, the joystick inputs specify the velocity of the
   * robot in the frame of reference of the robot.
   */

  public void disableFieldRelative() {
    this.isFieldRelative = false;
  }

  /**
   * Enables slow mode for translation. When enabled, the robot's translational velocities will be
   * scaled down.
   */

  public void enableTranslationSlowMode() {
    this.isTranslationSlowMode = true;
  }

  /**
   * Disables slow mode for translation. When disabled, the robot's translational velocities will
   * not be scaled.
   */

  public void disableTranslationSlowMode() {
    this.isTranslationSlowMode = false;
  }

  /**
   * Enables slow mode for rotation. When enabled, the robot's rotational velocity will be scaled
   * down.
   */

  public void enableRotationSlowMode() {
    this.isRotationSlowMode = true;
  }

  /**
   * Disables slow mode for rotation. When disabled, the robot's rotational velocity will not be
   * scaled.
   */

  public void disableRotationSlowMode() {
    this.isRotationSlowMode = false;
  }

  /**
   * Sets the robot's center of rotation. The origin is at the center of the robot. The positive x
   * direction is forward; the positive y direction, left.
   *
   * @param x the x coordinate of the robot's center of rotation (in meters)
   * @param y the y coordinate of the robot's center of rotation (in meters)
   */

  public void setCenterOfRotation(double x, double y) {
    io.setCenterOfRotation(new Translation2d(x, y));
  }

  /** Resets the robot's center of rotation to the center of the robot. */
  public void resetCenterOfRotation() {
    setCenterOfRotation(0.0, 0.0);
  }

  /**
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */

  public double getAverageDriveCurrent() {
    return this.inputs.drivetrain.averageDriveCurrent;
  }

  /**
   * Get the position of all drive wheels in radians.
   *
   * @return the position of all drive wheels in radians
   */

  public double[] getWheelRadiusCharacterizationPosition() {
    double[] positions = new double[inputs.swerve.length];
    for (int i = 0; i < inputs.swerve.length; i++) {
      positions[i] = inputs.drivetrain.swerveModulePositions[i].distanceMeters / (RobotConfiguration.getInstance().getWheelRadius().in(Meters));
    }

    return positions;
  }

  /**
   * Enables or disables the move-to-pose feature. Refer to the MoveToPose command class for more
   * information.
   *
   * @param state true to enable, false to disable
   */

  public void enableMoveToPose(boolean state) {
    this.isMoveToPoseEnabled = state;
  }

  /**
   * Returns true if the move-to-pose feature is enabled; false otherwise.
   *
   * @return true if the move-to-pose feature is enabled; false otherwise
   */

  public boolean isMoveToPoseEnabled() {
    return this.isMoveToPoseEnabled;
  }

  /**
   * Captures the initial positions of the drive wheels. This method is intended to be invoked at
   * the start of an autonomous path to measure the distance traveled by the robot.
   */

  public void captureInitialConditions() {
    for (int i = 0; i < this.inputs.swerve.length; i++) {
      this.initialDistance[i] = inputs.drivetrain.swerveModulePositions[i].distanceMeters;
    }
  }

  /**
   * Captures the final positions of the drive wheels and the final pose of the robot. This method
   * is intended to be invoked at the end of an autonomous path to measure the distance traveled by
   * the robot and the final pose of the robot. It logs the difference between the pose and the
   * final target pose of the specified autonomous path. If also logs the distance traveled by the
   * robot during the autonomous path.
   *
   * @param autoName the name of the autonomous path
   * @param measureDistance true to measure the distance traveled by the robot; false otherwise
   */

  public void captureFinalConditions(String autoName, boolean measureDistance) {
    try {
      List<Pose2d> pathPoses = PathPlannerPath.fromPathFile(autoName).getPathPoses();
      Pose2d targetPose = pathPoses.get(pathPoses.size() - 1);
      Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/AutoPoseDiff", targetPose.minus(this.customPose));

      if (measureDistance) {
        double distance = 0.0;
        for (int i = 0; i < this.inputs.swerve.length; i++) {
          distance += Math.abs(inputs.drivetrain.swerveModulePositions[i].distanceMeters - this.initialDistance[i]);
        }

        distance /= this.inputs.swerve.length;
        Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/AutoDistanceDiff", distance);
      }
    } catch (Exception e) {
      pathFileMissingAlert.setText("Could not find the specified path file: " + autoName);
      pathFileMissingAlert.set(true);
    }
  }

  /**
   * This method should be invoked once the alliance color is known. Refer to the RobotContainer's
   * checkAllianceColor method for best practices on when to check the alliance's color. The
   * alliance color is needed when running auto paths as those paths are always defined for
   * blue-alliance robots and need to be flipped for red-alliance robots.
   *
   * @param newAlliance the new alliance color
   */

  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  /**
   * Returns true if the auto path, which is always defined for a blue alliance robot, should be
   * flipped to the red alliance side of the field.
   *
   * @return true if the auto path should be flipped to the red alliance side of the field
   */

  public boolean shouldFlipAutoPath() {
    return this.alliance == Alliance.Red;
  }

  /**
   * If the robot is enabled and brake mode is not enabled, enable it. If the robot is disabled, has
   * stopped moving for the specified period of time, and brake mode is enabled; disable it.
   */

  private void updateBrakeMode() {
    if (DriverStation.isEnabled()) {
      if (!brakeMode) {
        brakeMode = true;
        setBrakeMode(true);
      }

      brakeModeTimer.restart();

    } else if (!DriverStation.isEnabled()) {
      boolean stillMoving = false;
      double velocityLimit = RobotConfiguration.getInstance().getRobotMaxCoastVelocity().in(MetersPerSecond);
      if (Math.abs(this.inputs.drivetrain.measuredChassisSpeeds.vxMetersPerSecond) > velocityLimit || Math.abs(this.inputs.drivetrain.measuredChassisSpeeds.vyMetersPerSecond) > velocityLimit) {
        stillMoving = true;
        brakeModeTimer.restart();
      }

      if (brakeMode && !stillMoving && brakeModeTimer.hasElapsed(BREAK_MODE_DELAY_SEC)) {
        brakeMode = false;
        setBrakeMode(false);
      }
    }
  }

  private void setBrakeMode(boolean enable) {
    this.io.setBrakeMode(enable);
  }

  public void enableRotationOverride() {
    this.isRotationOverrideEnabled = true;
  }

  public void disableRotationOverride() {
    this.isRotationOverrideEnabled = false;
  }

  /*
   * Enable and disable acceleration limiting
   */

  public void enableAccelerationLimiting() {
    this.accelerationLimiting = true;
  }

  public void disableAccelerationLimiting() {
    this.accelerationLimiting = false;
  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    // Some condition that should decide if we want to override rotation
    if (this.isRotationOverrideEnabled) {
      Rotation2d targetRotation = new Rotation2d();
      Logger.recordOutput(DrivetrainConstants.kSubsystemName + "/rotationOverride", targetRotation);
      return Optional.of(targetRotation);
    } else {
      // Return an empty optional when we don't want to override the path's rotation
      return Optional.empty();
    }
  }

  public Pose2d getFutureRobotPose(double translationSecondsInFuture, double rotationSecondsInFuture) {
    // project the robot pose into the future based on the current translational velocity; don't
    // project the current rotational velocity as that will adversely affect the control loop
    // attempting to reach the rotational setpoint.
    return this.getPose().exp(new Twist2d(
      this.getRobotRelativeSpeeds().vxMetersPerSecond * translationSecondsInFuture,
      this.getRobotRelativeSpeeds().vyMetersPerSecond * translationSecondsInFuture,
      this.getRobotRelativeSpeeds().omegaRadiansPerSecond * rotationSecondsInFuture
    ));
  }

  public Pose2d getCustomEstimatedPose() {
    return this.customPose;
  }

  public void resetCustomPose(Pose2d poseMeters) {
    this.io.resetPose(poseMeters);
  }

  public Optional<Pose2d> samplePoseAt(double timestamp) {
    return this.io.samplePoseAt(timestamp);
  }

  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
    this.io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  private void applySysIdCharacterization(SysIDCharacterizationMode mode, double value) {
    this.io.applySysIdCharacterization(mode, value);
  }

  public void setDriveToPoseCanceled(boolean canceled) {
    this.driveToPoseCanceled = canceled;
  }

  public boolean getDriveToPoseCanceled() {
    return this.driveToPoseCanceled;
  }

  public boolean isTilted() {
    boolean isTilted = Math.abs(this.inputs.drivetrain.rollDeg) > DrivetrainConstants.kTiltThresholdDegrees || Math.abs(this.inputs.drivetrain.pitchDeg) > DrivetrainConstants.kTiltThresholdDegrees;
    return isTilted;
  }

  public void untilt() {
    double roll = Units.degreesToRadians(this.inputs.drivetrain.rollDeg);
    double pitch = Units.degreesToRadians(this.inputs.drivetrain.pitchDeg);

    double gravityX = (9.8 * Math.cos(pitch) * Math.cos(roll) * Math.sin(pitch) * Math.cos(pitch));
    double gravityY = (-9.8 * Math.cos(pitch) * Math.cos(roll) * Math.sin(roll));

    double heading = Math.atan2(gravityY, gravityX);
    double xVelocity = Math.cos(heading) * DrivetrainConstants.kUntiltingVelocityMetersPerSecond;
    double yVelocity = Math.sin(heading) * DrivetrainConstants.kUntiltingVelocityMetersPerSecond;

    this.drive(xVelocity, yVelocity, 0.0, false, false);
  }

  public void sendWidget(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule rearLeft, SwerveModule rearRight, PigeonIMU gyro) {
    SmartDashboard.putData("Swerve Drive", new Sendable() {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> frontLeft.getCurrentState().angle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> frontLeft.getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Front Right Angle", () -> frontRight.getCurrentState().angle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> frontRight.getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Left Angle", () -> rearLeft.getCurrentState().angle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> rearLeft.getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Back Right Angle", () -> rearRight.getCurrentState().angle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> rearRight.getCurrentState().speedMetersPerSecond, null);

            builder.addDoubleProperty("Robot Angle", () -> Conversions.degreesToRadians(gyro.getFusedHeading()), null);
        }
    });
  }
}