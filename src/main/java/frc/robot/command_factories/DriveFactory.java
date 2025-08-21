package frc.robot.command_factories;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.minolib.swerve.MinoSwerveTeleopControl;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveFactory extends Command {
    private final DrivetrainSubsystem m_swerve;
    private final XboxController m_xboxController;
    private final MinoSwerveTeleopControl m_teleopControl;

    public DriveFactory(DrivetrainSubsystem swerve, XboxController xboxController) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);

        m_swerve = swerve;
        m_xboxController = xboxController;
        m_teleopControl = new MinoSwerveTeleopControl(
            DrivetrainConstants.linearSlewRate,
            DrivetrainConstants.angularSlewRate,
            DrivetrainConstants.stickDeadband,
            DrivetrainConstants.maxDriveSpeed,
            DrivetrainConstants.maxAngularVelocity,
            true
        );
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Map joystick axes to velocities. Joystick +x/+y don't correspond to field +x/+y, so map them accordingly.
        final var alliance = DriverStation.getAlliance();
        final var driveVelocities = alliance.isPresent() && alliance.get() == Alliance.Blue
            ? m_teleopControl.getDriveVelocitiesFromJoysticks(-m_xboxController.getLeftY(), -m_xboxController.getLeftX(), -m_xboxController.getRightX())
            : m_teleopControl.getDriveVelocitiesFromJoysticks(m_xboxController.getLeftY(), m_xboxController.getLeftX(), -m_xboxController.getRightX());

        m_swerve.driveOpenLoop(driveVelocities.xVel, driveVelocities.yVel, driveVelocities.thetaVel, true, DrivetrainConstants.teleopScrubLimit);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}