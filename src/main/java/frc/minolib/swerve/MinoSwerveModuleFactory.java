package frc.minolib.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import frc.minolib.hardware.CANDeviceID;
import frc.minolib.phoenix.MechanismRatio;
import frc.minolib.phoenix.PIDConfiguration;

public class MinoSwerveModuleFactory {
  private final double wheelCircumference;
  private final double steerDriveCouplingRatio;

  private final MechanismRatio driveRatio;
  private final MechanismRatio steeringRatio;

  private final PIDConfiguration driveOpenLoopPIDConfiguration;
  private final PIDConfiguration driveClosedLoopPIDConfiguration;
  private final PIDConfiguration steeringPIDConfiguration;

  public MinoSwerveModuleFactory(final double wheelCircumference, final MechanismRatio driveRatio, final MechanismRatio steeringRatio, final double steerDriveCouplingRatio, final PIDConfiguration driveOpenLoopPIDConfiguration, final PIDConfiguration driveClosedLoopPIDConfig, final PIDConfiguration steeringPIDConfig) {
    this.wheelCircumference = wheelCircumference;
    this.steeringRatio = steeringRatio;
    this.driveRatio = driveRatio;
    this.steerDriveCouplingRatio = steerDriveCouplingRatio;
    this.driveOpenLoopPIDConfiguration = driveOpenLoopPIDConfiguration;
    driveClosedLoopPIDConfiguration = driveClosedLoopPIDConfig;
    steeringPIDConfiguration = steeringPIDConfig;
  }

  public MinoSwerveModule createModule(final Translation2d position, final CANDeviceID driveMotorID, final CANDeviceID steeringMotorID, final CANDeviceID absEncoderID, final double absEncoderOffsetRad) {
    return new MinoSwerveModule(
      position,
      driveMotorID,
      steeringMotorID,
      absEncoderID,
      driveOpenLoopPIDConfiguration,
      driveClosedLoopPIDConfiguration,
      steeringPIDConfiguration,
      driveRatio,
      steeringRatio,
      steerDriveCouplingRatio,
      absEncoderOffsetRad,
      wheelCircumference
    );
  }
}