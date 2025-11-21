package frc.robot.subsystems.elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.naming.ldap.HasControls;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.minolib.advantagekit.LoggedTunableNumber;
import frc.minolib.math.EqualsUtility;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.GlobalConstants.Mode;
import lombok.Getter;
import lombok.Setter;

public class ElevatorSubsystem extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private BooleanSupplier coastOverride = () -> false;
    private BooleanSupplier disabledOverride = () -> false;

    @AutoLogOutput private boolean brakeModeEnabled = true;

    private TrapezoidProfile profile;
    @Getter private State setpoint = new State();
    private Supplier<State> goal = State::new;
    private boolean stopProfile = false;
    @Getter private boolean shouldEStop = false;
    @Setter private boolean isEStopped = false;
    @Setter private boolean forceFastConstraints = false;

    @AutoLogOutput(key = "Elevator/HomedPositionRotations")
    private double homedPosition = 0.0;

    @AutoLogOutput @Getter private boolean homed = false;

    @Getter
    @AutoLogOutput(key = "Elevator/Profile/AtGoal")
    private boolean atGoal = false;

    @Setter private boolean stowed = false;

    // Shove this later in the constants file
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD");

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 2.0);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kV", 4.0);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 3.0);

    private static final LoggedTunableNumber homingVoltage = new LoggedTunableNumber("Elevator/HomingVoltage", -4.0);
    private static final LoggedTunableNumber homingTimeSeconds = new LoggedTunableNumber("Elevator/HomingTimeSeconds", 0.1);
    private static final LoggedTunableNumber homingVelocityThreshold = new LoggedTunableNumber("Elevator/HomingVelocityThreshold", 0.6);
    private static final LoggedTunableNumber stowStopCheckHeight = new LoggedTunableNumber("Elevator/StowStopCheckHeight", .08);
    private static final LoggedTunableNumber tolerance = new LoggedTunableNumber("Elevator/Tolerance", 0.8);

    private static final LoggedTunableNumber maxVelocityMetersPerSecond = new LoggedTunableNumber("Elevator/MaxVelocityMetersPerSec", 2.0);
    private static final LoggedTunableNumber maxAccelerationMetersPerSecondSquared =new LoggedTunableNumber("Elevator/MaxAccelerationMetersPerSec2", 4.0);

    // Not from below here however
    private Debouncer homingDebouncer = new Debouncer(homingTimeSeconds.get());
    private Debouncer toleranceDebouncer = new Debouncer(0.25, DebounceType.kRising);

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocityMetersPerSecond.get(), maxAccelerationMetersPerSecondSquared.get()));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if(kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
            io.setPID(kP.get(), 0.0, kD.get());
        }

        if (maxVelocityMetersPerSecond.hasChanged(hashCode()) || maxAccelerationMetersPerSecondSquared.hasChanged(hashCode())) {
            profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocityMetersPerSecond.get(), maxAccelerationMetersPerSecondSquared.get()));
        }

        setBrakeMode(!coastOverride.getAsBoolean());

        final boolean shouldRunProfile =
            !stopProfile
            && !coastOverride.getAsBoolean()
            && !disabledOverride.getAsBoolean()
            && (homed || GlobalConstants.kCurrentMode == Mode.SIM)
            && !isEStopped
            && DriverStation.isEnabled();
        
        boolean outOfTolerance = Math.abs(getPositionMeters() - setpoint.position) > tolerance.get();
        shouldEStop = toleranceDebouncer.calculate(outOfTolerance && shouldRunProfile);

        if (shouldRunProfile) {
            var goalState = new State(
                MathUtil.clamp(goal.get().position, 0.0, Units.inchesToMeters(100)),
                goal.get().velocity
            );

            double previousVelocity = setpoint.velocity;
            setpoint = profile.calculate(GlobalConstants.kLoopPeriodSeconds, setpoint, goalState);

            if (setpoint.position < 0.0 || setpoint.position > Units.inchesToMeters(100)) {
                setpoint = new State(MathUtil.clamp(setpoint.position, 0.0, Units.inchesToMeters(100)), 0.0);
            }

            atGoal = EqualsUtility.epsilonEquals(setpoint.position, goalState.position) && EqualsUtility.epsilonEquals(setpoint.velocity, goalState.velocity);

            // Run
            if (stowed && atGoal && getPositionMeters() < stowStopCheckHeight.get()) {
                io.setDutyCycle(0.0);
            } else {
                double accel = (setpoint.velocity - previousVelocity) / GlobalConstants.kLoopPeriodSeconds;
                io.setTargetPosition(setpoint.position / homedPosition); // Maybe add FF here to update?
            }

            Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", setpoint.position);
            Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", setpoint.velocity);
            Logger.recordOutput("Elevator/Profile/GoalPositionMeters", goalState.position);
            Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", goalState.velocity);
        } else {
            // Reset setpoint
            setpoint = new State(getPositionMeters(), 0.0);

            Logger.recordOutput("Elevator/Profile/SetpointPositionMeters", 0.0);
            Logger.recordOutput("Elevator/Profile/SetpointVelocityMetersPerSec", 0.0);
            Logger.recordOutput("Elevator/Profile/GoalPositionMeters", 0.0);
            Logger.recordOutput("Elevator/Profile/GoalVelocityMetersPerSec", 0.0);
        }

        if (isEStopped) {
            io.setDutyCycle(0.0);
        }

        Logger.recordOutput("Elevator/ShouldRunProfile", shouldRunProfile);
    }

    @AutoLogOutput(key = "Elevator/MeasuredHeightMeters")
    public double getPositionMeters() {
        return (inputs.elevatorPositionInMeters - homedPosition);
    }

    public double getGoalMeters() {
        return goal.get().position;
    }

    public void setGoal(DoubleSupplier goal) {
        setGoal(() -> new State(goal.getAsDouble(), 0.0));
    }

    public void setGoal(Supplier<State> goal) {
        atGoal = false;
        this.goal = goal;
    }

    public void setOverrides(BooleanSupplier coastOverride, BooleanSupplier disabledOverride) {
        this.coastOverride = coastOverride;
        this.disabledOverride = disabledOverride;
    }

    private void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled) return;
        brakeModeEnabled = enabled;
        io.setNeutralMode(brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setHome() {
        homedPosition = inputs.elevatorPositionInMeters;
        homed = true;
    }

    public Command homingSequence() {
        return Commands.startRun(() -> {
            stopProfile = true;
            homed = false;
            homingDebouncer = new Debouncer(homingTimeSeconds.get());
            homingDebouncer.calculate(false);
        }, () -> {
        if (disabledOverride.getAsBoolean() || coastOverride.getAsBoolean()) return;
            io.setDutyCycle(homingVoltage.get());
            homed = homingDebouncer.calculate(Math.abs(inputs.elevatorVelocityMetersPerSecond) <= homingVelocityThreshold.get());
        }).until(() -> homed).andThen(this::setHome).finallyDo(() -> {
            stopProfile = false;
        });
    }
}
