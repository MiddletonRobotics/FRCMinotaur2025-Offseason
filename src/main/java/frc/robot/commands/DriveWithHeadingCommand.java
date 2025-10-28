package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.minolib.math.EqualsUtility;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GlobalConstants;
import frc.robot.subsystems.drivetrain.CompetitionTunerConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;

public class DriveWithHeadingCommand extends Command {

    protected DrivetrainSubsystem drivetrain;
    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private Optional<Rotation2d> headingSetpoint = Optional.empty();
    private double joystickLastTouched = -1;

    private final SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
        .withDeadband(CompetitionTunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.05) // Add a 5% deadband in open loop
        .withRotationalDeadband(DrivetrainConstants.kDriveMaxAngularRate * GlobalConstants.kControllerDeadband)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(CompetitionTunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.05)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    public DriveWithHeadingCommand(DrivetrainSubsystem drivetrain, DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drivetrain = drivetrain;
        this.throttleSupplier = throttle;
        this.strafeSupplier = strafe;
        this.rotationSupplier = turn;

        driveWithHeading.HeadingController.setPID(
            DrivetrainConstants.kHeadingControllerP,
            DrivetrainConstants.kHeadingControllerI,
            DrivetrainConstants.kHeadingControllerD
        );

        addRequirements(drivetrain);
        setName("Swerve Drive Maintain Heading");

        if (Robot.isSimulation()) {
            driveNoHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
            driveWithHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        }
    }

    @Override
    public void initialize() {
        headingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = throttleSupplier.getAsDouble() * CompetitionTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double strafe = strafeSupplier.getAsDouble() * CompetitionTunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        double turnFieldFrame = rotationSupplier.getAsDouble();
        double throttleFieldFrame = GlobalConstants.isBlueAlliance() ? throttle : -throttle;
        double strafeFieldFrame = GlobalConstants.isBlueAlliance() ? strafe : -strafe;

        if (Math.abs(turnFieldFrame) > GlobalConstants.kControllerDeadband) {
            joystickLastTouched = Timer.getFPGATimestamp();
        }

        if (Math.abs(turnFieldFrame) > GlobalConstants.kControllerDeadband || (EqualsUtility.epsilonEquals(joystickLastTouched, Timer.getFPGATimestamp(), 0.25)
            && Math.abs(drivetrain.getRobotRelativeChassisSpeeds().omegaRadiansPerSecond) > Math.toRadians(10))) {

            drivetrain.setControl((driveNoHeading
                .withVelocityX(throttleFieldFrame)
                .withVelocityY(strafeFieldFrame)
                .withRotationalRate(turnFieldFrame * DrivetrainConstants.kDriveMaxAngularRate)));

            headingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (headingSetpoint.isEmpty()) {
                headingSetpoint =  Optional.of(drivetrain.getPose().getRotation());
            }

            Logger.recordOutput("DriveMaintainHeading/throttleFieldFrame", throttleFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/strafeFieldFrame", strafeFieldFrame);
            Logger.recordOutput("DriveMaintainHeading/mHeadingSetpoint", headingSetpoint.get());

            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput("DriveMaintainHeading/HeadingSetpoint", headingSetpoint.get().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
