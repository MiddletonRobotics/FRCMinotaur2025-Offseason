package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.MagnetHealthValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.utilities.constants.DrivetrainConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveSimulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.025, DrivetrainConstants.kDriveGearRatio), DCMotor.getKrakenX60(1));
    private DCMotorSim steerSimulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, DrivetrainConstants.kTurnGearRatio), DCMotor.getKrakenX60(1));

    private boolean driveClosedLoop = false;
    private boolean steerClosedLoop = false;

    private PIDController driveController = new PIDController(0.0, 0.0, 0.0);
    private PIDController steerController = new PIDController(0.0, 0.0, 0.0);

    private double driveAppliedVoltage = 0.0;
    private double driveFFVoltage = 0.0;
    private double steerAppliedVoltage = 0.0;

    public ModuleIOSim() {
        steerController.enableContinuousInput(-Math.PI, Math.PI);
      }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if(driveClosedLoop) {
            driveAppliedVoltage = driveFFVoltage + driveController.calculate(driveSimulation.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }

        if(steerClosedLoop) {
            steerAppliedVoltage = steerController.calculate(steerSimulation.getAngularPositionRad());
        } else {
            steerController.reset();
        }

        driveSimulation.setInputVoltage(MathUtil.clamp(driveAppliedVoltage, -12.0, 12.0));
        driveSimulation.update(LOOP_PERIOD_SECS);
        steerSimulation.setInputVoltage(MathUtil.clamp(steerAppliedVoltage, -12.0, 12.0));
        steerSimulation.update(LOOP_PERIOD_SECS);

        inputs.isDriveMotorConnected = true;
        inputs.drivePositionRadians = driveSimulation.getAngularPositionRad();
        inputs.driveVelocityRadiansPerSecond = driveSimulation.getAngularVelocityRadPerSec();
        inputs.driveAccelerationRadiansPerSecondSquared = driveSimulation.getAngularAccelerationRadPerSecSq();
        inputs.driveAppliedVoltage = driveAppliedVoltage;
        inputs.driveCurrentAmperes = new double[] {Math.abs(driveSimulation.getCurrentDrawAmps())};

        inputs.isSteerMotorConnected = true;
        inputs.steerPosition = new Rotation2d(steerSimulation.getAngularPositionRad());
        inputs.steerVelocityRadiansPerSecond = steerSimulation.getAngularVelocityRadPerSec();
        inputs.steerAccelerationRadiansPerSecondSquared = steerSimulation.getAngularAccelerationRadPerSecSq();
        inputs.steerAppliedVoltage = steerAppliedVoltage;
        inputs.steerCurrentAmperes = new double[] {Math.abs(steerSimulation.getCurrentDrawAmps())};

        inputs.isSwerveEncoderConnected = true;
        inputs.swerveEncoderMagnetHealth = MagnetHealthValue.Magnet_Green;
        inputs.swerveEncoderPosition = new Rotation2d(steerSimulation.getAngularPositionRad());

        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRadians = new double[] {inputs.drivePositionRadians};
        inputs.odometrySteerPositions = new Rotation2d[] {inputs.steerPosition};
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVoltage = output * 12.0;
    }

    @Override
    public void setSteerOpenLoop(double output) {
        steerClosedLoop = false;
        steerAppliedVoltage = output * 12.0;
    }   

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
        driveClosedLoop = true;
        driveFFVoltage = feedforward;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setSteerPosition(Rotation2d rotation) {
        steerClosedLoop = true;
        steerController.setSetpoint(rotation.getRadians());
    }

    @Override
    public void setDrivePIDCoefficients(double kP, double kI, double kD) {
        driveController.setPID(kP, kI, kD);
    }

    @Override
    public void setSteerPIDCoefficients(double kP, double kI, double kD) {
        steerController.setPID(kP, kI, kD);
    }
}