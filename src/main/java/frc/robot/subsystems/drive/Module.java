package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utilities.AlertManager;
import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.constants.GlobalConstants;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final SimpleMotorFeedforward driveFeedforward;
    private boolean steerRelativeReset = false;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private Alert driveDisconnectedAlert;
    private Alert steerDisconnectedAlert;
    private Alert swerveEncoderDisconnectedAlert;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        if(RobotBase.isReal()) {
            driveFeedforward = new SimpleMotorFeedforward(0.21125, 0.13448);
            io.setDrivePID(2.0, 0.0 , 0.0);
            io.setSteerPID(300.0, 0.0, 0.0);
        } else {
            driveFeedforward = new SimpleMotorFeedforward(0.0, 0.13);
            io.setDrivePID(0.1, 0.0, 0.0);
            io.setSteerPID(10.0, 0.0, 0.0);
        }

        this.driveDisconnectedAlert = new Alert(String.format("Drive %d Disconnected", index), Alert.AlertType.kError);
        this.steerDisconnectedAlert = new Alert(String.format("Steer %d Disconnected", index), Alert.AlertType.kError);
        this.swerveEncoderDisconnectedAlert = new Alert(String.format("Swerve Encoder %d Disconnected", index), Alert.AlertType.kError);

        AlertManager.registerAlert(driveDisconnectedAlert, steerDisconnectedAlert, swerveEncoderDisconnectedAlert);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/Module" +Integer.toString(index), inputs);
    }

    public void periodic() {
        if(steerRelativeReset && inputs.isSteerEncoderConnected) {
            io.resetSteerMotor(inputs.steerAbsolutePosition.getMeasure());
            steerRelativeReset = true;
        }

        int sampleCount = inputs.odometryTimestamps.length;
        odometryPositions = new SwerveModulePosition[sampleCount];

        for(int i = 0; i < sampleCount; i++) {
            double positionMeters = inputs.odometryDrivePositionRadians[i] * DrivetrainConstants.kWheelRadius.in(Meters);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        if(GlobalConstants.kUseAlerts && !inputs.isDriveMotorConnected) {
            driveDisconnectedAlert.set(true);
        } else {
            driveDisconnectedAlert.set(false);
        }

        if(GlobalConstants.kUseAlerts && !inputs.isSteerMotorConnected) {
            steerDisconnectedAlert.set(true);
        } else {
            steerDisconnectedAlert.set(false);
        }

        if(GlobalConstants.kUseAlerts && !inputs.isSteerEncoderConnected) {
            swerveEncoderDisconnectedAlert.set(true);
        } else {
            swerveEncoderDisconnectedAlert.set(false);
        }
    }

    public Rotation2d getAngle() {
        return inputs.steerPositionRadians;
    }

    public double getPositionMeters() {
        return inputs.drivePositionRadians * DrivetrainConstants.kWheelRadius.in(Meters);
    }

    public double getVelocityMetersPerSecond() {
        return inputs.driveVelocityRadiansPerSecond * DrivetrainConstants.kWheelRadius.in(Meters);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());   
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSecond(), getAngle());
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestamps;
    }

    public double getDriveVelocity() {
        return inputs.driveVelocityRadiansPerSecond;
    }

    public double getWheelRadiusCharacterization() {
        return inputs.drivePositionRadians;
    }

    public double getDriveAcceleration() {
        return inputs.driveAccelerationRadiansPerSecondSquared;
    }

    public double getDriveCurrent() {
        return inputs.driveCurrentAmps;
    }

    public double getDriveTempurature() {
        return inputs.driveTempuratureCelsius;
    }

    public void setCurrentLimits(double supplyLimit) {
        io.setCurrentLimit(supplyLimit);
    }

    public void runSetpoint(SwerveModuleState state) {
        state.optimize(getAngle());

        double speedRadiansPerSecond = state.speedMetersPerSecond / DrivetrainConstants.kWheelRadius.in(Meters);

        io.setDriveVelocity(speedRadiansPerSecond, driveFeedforward.calculate(speedRadiansPerSecond));
        io.setSteerPosition(state.angle);

        Logger.recordOutput("Module" + index + "/DriveFF", driveFeedforward.calculate(speedRadiansPerSecond));
    }

    public void runCharacterization(double output) {
        io.setSteerPosition(new Rotation2d());
        io.setDriveRawOutput(output);
    }

    public void stop() {
        io.setDriveRawOutput(0.0);
        io.setSteerRawOutput(0.0);
    }

    public void setBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
        io.setSteerBrakeMode(enabled);
    }

    public void setDriveBrakeMode(boolean enabled) {
        io.setDriveBrakeMode(enabled);
    }
}
