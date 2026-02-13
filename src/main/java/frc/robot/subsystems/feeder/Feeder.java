// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private FeederWantedState wantedState = FeederWantedState.STOPPED;
  private FeederCurrentState currentState = FeederCurrentState.STOPPED;

  public enum FeederWantedState{
    STOPPED,
    FEED_FUEL,
    REVERSE,
  }

  public enum FeederCurrentState{
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
            currentState = FeederCurrentState.FEEDING_FUEL;
            break;
        case STOPPED:
            currentState = FeederCurrentState.STOPPED;
            break;
        case REVERSE:
            currentState = FeederCurrentState.REVERSE;
            break;
        default:
            currentState = FeederCurrentState.STOPPED;
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
        break;
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

  public void setWantedState(Feeder.FeederWantedState wantedState){
    this.wantedState = wantedState;
  }

}