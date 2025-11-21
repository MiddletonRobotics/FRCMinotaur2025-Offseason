package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.oi.Controlboard;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;

public class Superstructure extends SubsystemBase {
    private final DrivetrainSubsystem drivetrain;
    private final ElevatorSubsystem elevator;
    private final Controlboard controlboard;
    private final LedSubsystem led;

    private boolean hasPoseBeenResetPrematch = false;
    private boolean allowExternalCommandToAccessLEDS = false;

    public Superstructure(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, LedSubsystem led, Controlboard controlboard) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.controlboard = controlboard;
        this.led = led;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled() && !allowExternalCommandToAccessLEDS) {
            if(controlboard.povLeft().getAsBoolean()) {
                led.setWantedAction(LedSubsystem.WantedState.DISPLAY_CONTROLLERS_ACTIVE);
            } else {
                led.setWantedAction(hasPoseBeenResetPrematch ? LedSubsystem.WantedState.DISPLAY_READY_FOR_MATCH : LedSubsystem.WantedState.DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH);
            }
        } else if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            led.setWantedAction(LedSubsystem.WantedState.DISPLAY_OFF);
        }
    }

    public void toggleHasPoseBeenSetForPrematch(boolean hasBeenReset) {
        this.hasPoseBeenResetPrematch = hasBeenReset;
    }

    public void setExternalCommandAllowedToControlLEDs(boolean allowed) {
        this.allowExternalCommandToAccessLEDS = allowed;
    }
}
