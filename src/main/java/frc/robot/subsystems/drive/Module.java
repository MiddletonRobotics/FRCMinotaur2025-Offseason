package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Robot;
import frc.robot.utilities.AlertManager;
import frc.robot.utilities.LoggedTracer;
import frc.robot.utilities.LoggedTunableNumber;
import frc.robot.utilities.constants.DrivetrainConstants;
import frc.robot.utilities.constants.GlobalConstants;

public class Module {
    private static final LoggedTunableNumber drivekS = new LoggedTunableNumber("Drive/Module/DrivekS");
    private static final LoggedTunableNumber drivekV = new LoggedTunableNumber("Drive/Module/DrivekV");
    private static final LoggedTunableNumber drivekT = new LoggedTunableNumber("Drive/Module/DrivekT");
    private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Drive/Module/DrivekP");
    private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Drive/Module/DrivekD");
    private static final LoggedTunableNumber steerkP = new LoggedTunableNumber("Drive/Module/TurnkP");
    private static final LoggedTunableNumber steerkD = new LoggedTunableNumber("Drive/Module/TurnkD");

    static {
        switch (GlobalConstants.kCurrentMode) {
            case REAL -> {
                drivekS.initDefault(5.0);
                drivekV.initDefault(0.0);
                drivekT.initDefault(DrivetrainConstants.kDriveGearRatio / DCMotor.getKrakenX60(1).KtNMPerAmp);
                drivekP.initDefault(35.0);
                drivekD.initDefault(0.0);
                steerkP.initDefault(4000.0);
                steerkD.initDefault(50.0);
            }

            default -> {
                drivekS.initDefault(0.014);
                drivekV.initDefault(0.134);
                drivekT.initDefault(0);
                drivekP.initDefault(0.1);
                drivekD.initDefault(0);
                steerkP.initDefault(10.0);
                steerkD.initDefault(0);
            }
        }
    }

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private SimpleMotorFeedforward driveFeedforward;
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

    private final Debouncer driveMotorConnectedDebouncer =  new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer steerMotorConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer swerveEncoderConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

    private final Alert driveDisconnectedAlert;
    private final Alert steerDisconnectedAlert;
    private final Alert swerveEncoderDisonnectedAlert;

    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

        driveFeedforward = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());

        driveDisconnectedAlert = new Alert("Swerve Module [" + index + "] drive motor is disconnected", AlertType.kError);
        steerDisconnectedAlert = new Alert("Swerve Module [" + index + "] steer motor is disconnected" + index + " is disconnected.", AlertType.kError);
        swerveEncoderDisonnectedAlert = new Alert("Swerve Module [" + index + "] swerve encoder is disconnected", AlertType.kError);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drivetrain/Module" + index, inputs);
    }

    public void periodic() {
        if(drivekS.hasChanged(hashCode()) || drivekV.hasChanged(hashCode())) {
            driveFeedforward = new SimpleMotorFeedforward(drivekS.get(), drivekV.get());
        }

        if(drivekP.hasChanged(hashCode()) || drivekD.hasChanged(hashCode())) {
            io.setDrivePIDCoefficients(drivekP.get(), 0.0, drivekD.get());
        }

        if(steerkP.hasChanged(hashCode()) || steerkD.hasChanged(hashCode())) {
            io.setSteerPIDCoefficients(steerkP.get(), 0.0, steerkD.get());
        }

        int sampleCounts = inputs.odometryDrivePositionsRadians.length;
        odometryPositions = new SwerveModulePosition[sampleCounts];
        for (int i = 0; i < sampleCounts; i++) {
            double positionMeters = inputs.odometryDrivePositionsRadians[i] * DrivetrainConstants.kWheelRadius.in(Meters);
            Rotation2d angle = inputs.odometrySteerPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }

        driveDisconnectedAlert.set(!driveMotorConnectedDebouncer.calculate(inputs.isDriveMotorConnected) && !Robot.isJITing());
        steerDisconnectedAlert.set(!steerMotorConnectedDebouncer.calculate(inputs.isSteerMotorConnected) && !Robot.isJITing());
        swerveEncoderDisonnectedAlert.set(!swerveEncoderConnectedDebouncer.calculate(inputs.isSwerveEncoderConnected) && !Robot.isJITing());

        LoggedTracer.record("Drivetrain/Module" + index);
    }

    public void runSetpoint(SwerveModuleState state) {
        double speedRadiansPerSecond = state.speedMetersPerSecond / DrivetrainConstants.kWheelRadius.in(Meters);
        io.setDriveVelocity(speedRadiansPerSecond, driveFeedforward.calculate(speedRadiansPerSecond));
        io.setSteerPosition(state.angle);
    }

    public void runSetpoint(SwerveModuleState state, double wheelTorqueNm) {
        double speedRadiansPerSecond = state.speedMetersPerSecond / DrivetrainConstants.kWheelRadius.in(Meters);
        io.setDriveVelocity(wheelTorqueNm, driveFeedforward.calculate(speedRadiansPerSecond) + wheelTorqueNm * drivekT.get());
        io.setSteerPosition(state.angle);
    }

    public void runCharacterization(double output) {
        io.setDriveOpenLoop(output);
        io.setSteerPosition(Rotation2d.kZero);
    }

    public void stop() {
        io.setDriveOpenLoop(0.0);
        io.setSteerOpenLoop(0.0);
    }

    public Rotation2d getAngle() {
        return inputs.steerPosition;
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

    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRadians;
    }

    public double getFFCharacterizationVelocity() {
        return Units.radiansToRotations(inputs.driveVelocityRadiansPerSecond);
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }
}
