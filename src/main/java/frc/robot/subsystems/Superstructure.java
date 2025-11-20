package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.oi.Controlboard;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.LedSubsystem.WantedState;

public class Superstructure extends SubsystemBase {
    private final DrivetrainSubsystem drivetrain;
    private final Controlboard controlboard;
    private final LedSubsystem led;

    public Superstructure(DrivetrainSubsystem drivetrain, LedSubsystem led, Controlboard controlboard) {
        this.drivetrain = drivetrain;
        this.controlboard = controlboard;
        this.led = led;
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            if(controlboard.povLeft().getAsBoolean()) {
                led.setWantedAction(WantedState.DISPLAY_CONTROLLERS_ACTIVE);
            } else {
                led.setWantedAction(LedSubsystem.WantedState.DISPLAY_ROBOT_IS_PHYSICALLY_READY_FOR_MATCH);
            }
        } else if (DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            led.setWantedAction(LedSubsystem.WantedState.DISPLAY_OFF);
        }
    }

    public void toggleHasPoseBeenSetForPrematch(boolean hasBeenReset) {

    }

    public void setExternalCommandAllowedToControlLEDs(boolean allowed) {

    }
}
