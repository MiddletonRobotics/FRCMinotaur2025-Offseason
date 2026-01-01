package frc.robot.subsystems.bucket;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import frc.minolib.advantagekit.LoggedTracer;
import frc.robot.Robot;
import lombok.Setter;

public class BucketSubsystem {
    private final String name;
    private final String inputsName;
    private final BucketIO io;
    protected final BucketIOInputsAutoLogged inputs = new BucketIOInputsAutoLogged();

    @Setter private double volts = 0.0;
    private boolean brakeModeEnabled = true;


    public BucketSubsystem(String name, String inputsName, BucketIO io) {
        this.name = name;
        this.inputsName = inputsName;
        this.io = io;
    }

    public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);

    // Run roller
    io.runVolts(volts);

    // Record cycle time
    LoggedTracer.record(name);
    Logger.recordOutput(inputsName + "/BrakeModeEnabled", brakeModeEnabled);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(enabled);
  }

  public double getSupplyCurrent() {
    return inputs.bucketSupplyCurrent;
  }

  public double getVelocity() {
    return inputs.bucketVelocity;
  }

  public void stop() {
    volts = 0.0;
  }
}
