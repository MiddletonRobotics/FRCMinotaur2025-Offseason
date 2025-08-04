package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

public class ModuleIOReplay implements ModuleIO {
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {}

    @Override
    public void setDriveVelocity(double velocityRadiansPerSecond, double feedforward) {}

    @Override
    public void setSteerPosition(Rotation2d position) {}

    @Override
    public void setDriveBrakeMode(boolean enable) {}

    @Override
    public void setSteerBrakeMode(boolean enable) {}

    @Override
    public void setCurrentLimit(double supplyLimit) {}

    @Override
    public void setDriveRawOutput(double output) {}

    @Override
    public void setSteerRawOutput(double output) {}

    @Override
    public void setDrivePID(double kP, double kI, double kD) {}

    @Override
    public void setSteerPID(double kP, double kI, double kD) {}

    @Override
    public void resetSteerMotor(Angle position) {}
}
