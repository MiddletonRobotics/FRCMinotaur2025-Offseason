package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class ModuleIOReplay implements ModuleIO {
    
    @Override
    public void updateInputs(ModuleIOInputs inputs) {}

    @Override
    public void setDriveOpenLoop(double output) {}

    @Override
    public void setSteerOpenLoop(double output) {}

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double feedforward) {}

    @Override
    public void setSteerPosition(Rotation2d rotation) {}

    @Override
    public void setDrivePIDCoefficients(double kP, double kI, double kD) {}

    @Override
    public void setSteerPIDCoefficients(double kP, double kI, double kD) {}

    @Override
    public void setBrakeMode(boolean enabled) {}
}
