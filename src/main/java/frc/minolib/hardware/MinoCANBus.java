package frc.minolib.hardware;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;

import frc.minolib.io.CANBusInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

/**
 * The MinoCANBus class represents a CAN bus interface for the robot. It provides functionality to
 * log and update the status of the CAN bus.
 *
 * <p>It includes an inner static class CANBusInputs which holds the status and various metrics of
 * the CAN bus.
 *
 * <p>Usage:
 *
 * <pre>{@code
 * MinoCANBus canBus = new MinoCANBus();
 * canBus.updateInputs();
 * }</pre>
 *
 * <p>Constructor Summary:
 *
 * <ul>
 *   <li>{@link #MinoCANBus()} - Initializes the CAN bus with the default name "rio".
 *   <li>{@link #MinoCANBus(String canbusName)} - Initializes the CAN bus with the specified name.
 * </ul>
 *
 * <p>Method Summary:
 *
 * <ul>
 *   <li>{@link #updateInputs()} - Updates the CAN bus inputs and logs the current status.
 * </ul>
 *
 * <p>Inner Class:
 *
 * <ul>
 *   <li>{@link MinoCANBus.CANBusInputs} - Represents the inputs for a CAN bus.
 * </ul>
 */
public class MinoCANBus {
  private final String canbusName;
  private final String loggingName;
  private final CANBus canBus;

  private final CANBusInputsAutoLogged inputs = new CANBusInputsAutoLogged();

  public MinoCANBus() {
    this("rio");
  }

  public MinoCANBus(final String canbusName) {
    this.canbusName = canbusName;
    loggingName = "Inputs/CANBus [" + canbusName + "]";
    canBus = new CANBus(this.canbusName);
  }

  public void updateInputs() {
    CANBusStatus status = canBus.getStatus();
    inputs.status = status.Status;
    inputs.busUtilization = status.BusUtilization;
    inputs.busOffCount = status.BusOffCount;
    inputs.txFullCount = status.TxFullCount;
    inputs.REC = status.REC;
    inputs.TEC = status.TEC;
    inputs.isNetworkFD = canBus.isNetworkFD();
    Logger.processInputs(loggingName, inputs);
  }
}