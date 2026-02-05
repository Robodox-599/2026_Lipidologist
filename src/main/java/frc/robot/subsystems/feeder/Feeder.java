// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPED;

  public enum WantedState{
    STOPPED,
    FEED_FUEL,
    REVERSE,
  }

  public enum CurrentState{
    STOPPED,
    FEEDING_FUEL,
    REVERSE,
  }

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
    
    handleFeederStateTransitions();
    applyStates();

    DogLog.log("Feeder/wantedState", wantedState);
    DogLog.log("Feeder/currentState", currentState);

  }

  public void handleFeederStateTransitions(){
    switch (wantedState) {
        case FEED_FUEL:
            currentState = CurrentState.FEEDING_FUEL;
            break;
        case STOPPED:
            currentState = CurrentState.STOPPED;
            break;
        case REVERSE:
            currentState = CurrentState.REVERSE;
            break;
        default:
            currentState = CurrentState.STOPPED;
            break;
    }
  }

  public void applyStates(){
    switch(currentState){
      case FEEDING_FUEL:
        setFeederVoltage(3);
        break;
      case STOPPED:
        stopFeeder();
        break;
      case REVERSE:
        setFeederVoltage(-3.0);
      default:
        stopFeeder();
        break;
    }
  }

  public void setFeederVoltage(double volts){
    io.setFeederVoltage(volts);
  }

  public void stopFeeder(){
    io.stopFeeder();
  }

  public void setWantedState(Feeder.WantedState wantedState){
    this.wantedState = wantedState;
  }

}