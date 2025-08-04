package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utilities.constants.DrivetrainConstants;

public class ModuleIOSim implements ModuleIO {
    private static final double LOOP_PERIOD_SECONDS = 0.02;

    private DCMotor driveMotor = DCMotor.getKrakenX60(1);
    private DCMotor steerMotor = DCMotor.getKrakenX60(1);

    private DCMotorSim driveSimulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(driveMotor, DrivetrainConstants.kDriveSimMOI, DrivetrainConstants.kDriveSimGearRatio), driveMotor);
    private DCMotorSim steerSimulation = new DCMotorSim(LinearSystemId.createDCMotorSystem(steerMotor, DrivetrainConstants.kTurnSimMOI, DrivetrainConstants.kTurnSimGearRatio), steerMotor);

    private final Rotation2d steerAbsoluteInitializePosition = new Rotation2d(Math.random() * 2 * Math.PI);
    private double driveAppliedVoltage = 0.0;
    private double driveFeedforward = 0.0;
    private double steerAppliedVoltage = 0.0;

    private PIDController drivePIDController = new PIDController(0.0, 0.0, 0.0);
    private PIDController steerPIDController = new PIDController(0.0, 0.0, 0.0);

    {
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private boolean driveVelocityControl = false;
    private boolean steerPositionControl = false;

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if(driveVelocityControl) {
            driveAppliedVoltage = drivePIDController.calculate(driveSimulation.getAngularVelocityRadPerSec()) + driveFeedforward;
        }

        if(steerPositionControl) {
            steerAppliedVoltage = steerPIDController.calculate(steerSimulation.getAngularPositionRad());
        }

        driveSimulation.setInputVoltage(driveAppliedVoltage);
        steerSimulation.setInputVoltage(steerAppliedVoltage);

        driveSimulation.update(LOOP_PERIOD_SECONDS);
        steerSimulation.update(LOOP_PERIOD_SECONDS);

        inputs.drivePositionRadians = driveSimulation.getAngularPositionRad();
        inputs.driveVelocityRadiansPerSecond = driveSimulation.getAngularVelocityRadPerSec();
        inputs.driveAccelerationRadiansPerSecondSquared = driveSimulation.getAngularAccelerationRadPerSecSq();
        inputs.driveAppliedVolts = driveAppliedVoltage;
        inputs.driveCurrentAmps = Math.abs(driveSimulation.getCurrentDrawAmps());
        
        inputs.steerAbsolutePosition = new Rotation2d(steerSimulation.getAngularPositionRad()).plus(steerAbsoluteInitializePosition);
        inputs.steerPositionRadians = new Rotation2d(steerSimulation.getAngularPositionRad());
        inputs.steerVelocityRadiansPerSecond = steerSimulation.getAngularVelocityRadPerSec();
        inputs.steerAccelerationRadiansPerSecondSquared = steerSimulation.getAngularAccelerationRadPerSecSq();
        inputs.steerAppliedVolts = steerAppliedVoltage;
        inputs.steerCurrentAmps = Math.abs(steerSimulation.getCurrentDrawAmps());

        inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometryDrivePositionRadians = new double[] { inputs.drivePositionRadians };
        inputs.odometrySteerPositions = new Rotation2d[] { inputs.steerAbsolutePosition };
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double feedforward) {
        drivePIDController.setSetpoint(velocityRadPerSec);
        driveFeedforward = feedforward;
        driveVelocityControl = true;
    }

    @Override
    public void setSteerPosition(Rotation2d position) {
        steerPIDController.setSetpoint(position.getRadians());
        steerPositionControl = true;
    }

    @Override
    public void setDriveBrakeMode(boolean enable) {}

    @Override
    public void setSteerBrakeMode(boolean enable) {}

    @Override
    public void setCurrentLimit(double supplyLimit) {}

    @Override
    public void setDriveRawOutput(double output) {
        driveAppliedVoltage = MathUtil.clamp(output, -12.0, 12.0);
        driveVelocityControl = false;
    }

    @Override
    public void setSteerRawOutput(double output) {
        steerAppliedVoltage = MathUtil.clamp(output, -12.0, 12.0);
        steerPositionControl = false;
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        drivePIDController.setPID(kP, kI, kD);
    }

    @Override
    public void setSteerPID(double kP, double kI, double kD) {
        steerPIDController.setPID(kP, kI, kD);
    }

    @Override
    public void resetSteerMotor(Angle position) {}
}
