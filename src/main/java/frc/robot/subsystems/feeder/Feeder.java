// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import java.util.spi.CurrencyNameProvider;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Tracer;

public class Feeder {
  private final FeederIO io;
  private FeederWantedState wantedState = FeederWantedState.STOPPED;
  private FeederCurrentState currentState = FeederCurrentState.STOPPED;

  public enum FeederWantedState {
    STOPPED,
    FEED_FUEL,
    CLEAR_JAMMED_FUEL,
    REVERSE,
    OUTAKE,
  }

  public enum FeederCurrentState {
    STOPPED,
    FEEDING_FUEL,
    CLEARING_JAMMED_FUEL,
    REVERSE,
    OUTAKING,
  }

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public void updateInputs() {
    Tracer.traceFunc("FeederUpdateInputs", io::updateInputs);

    handleFeederStateTransitions();
    applyStates();

    DogLog.log("Feeder/wantedState", wantedState);
    DogLog.log("Feeder/currentState", currentState);

  }

  public void handleFeederStateTransitions() {
    switch (wantedState) {
      case FEED_FUEL:
        if(isFeederStalling()) {
          currentState = FeederCurrentState.CLEARING_JAMMED_FUEL;
        } else {
          currentState = FeederCurrentState.FEEDING_FUEL;
        }
        break;
      case CLEAR_JAMMED_FUEL:
        if(clearingFuel()) {
          currentState = FeederCurrentState.FEEDING_FUEL;
        } else {
          currentState = FeederCurrentState.CLEARING_JAMMED_FUEL;
        }
        break;
      case STOPPED:
        currentState = FeederCurrentState.STOPPED;
        break;
      case REVERSE:
        currentState = FeederCurrentState.REVERSE;
        break;
      case OUTAKE:
        currentState = FeederCurrentState.OUTAKING;
        break;
      default:
        currentState = FeederCurrentState.STOPPED;
        break;
    }
  }

  public void applyStates() {
    switch (currentState) {
      case FEEDING_FUEL:
        setFeederVelocity(100);
        break;
      case CLEARING_JAMMED_FUEL:
        clearFuel();
        break;
      case STOPPED:
        stopFeeder();
        break;
      case REVERSE:
        setVoltage(-3);
        break;
      case OUTAKING:
        setVoltage(-12);
        break;
      default:
        stopFeeder();
        break;
    }
  }

  private void setFeederVelocity(double RPS) {
    io.setFeederVelocity(RPS);
  }

  private void clearFuel() {
    io.clearFuel();
  }

  private void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void stopFeeder() {
    io.stopFeeder();
  }

  public void setWantedState(Feeder.FeederWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean isFeederStalling() {
    return io.isFeederStalling;
  }

  public boolean clearingFuel() {
    return io.clearingFuel;
  }

}