// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private FeederWantedState feederWantedState = FeederWantedState.STOPPED;
  private FeederCurrentState feederCurrentState = FeederCurrentState.STOPPED;

  public enum FeederWantedState{
    STOPPED,
    TRANSFERING_FUEL,
    REVERSE,
  }

  public enum FeederCurrentState{
    STOPPED,
    TRANSFERING_FUEL,
    REVERSE,
  }

  public Feeder(FeederIO io) {
    this.io = io;
  }

  public void updateFeederInputs() {
    io.updateFeederInputs();
    
    handleFeederStateTransitions();
    applyStates();

    DogLog.log("Feeder/wantedState", feederWantedState);
    DogLog.log("Feeder/currentState", feederCurrentState);

  }

  public void handleFeederStateTransitions(){
    switch (feederWantedState) {
        case TRANSFERING_FUEL:
            feederCurrentState = FeederCurrentState.TRANSFERING_FUEL;
            break;
        case STOPPED:
            feederCurrentState = FeederCurrentState.STOPPED;
            break;
        case REVERSE:
            feederCurrentState = FeederCurrentState.REVERSE;
            break;
        default:
            feederCurrentState = FeederCurrentState.STOPPED;
            break;
    }
  }

  public void applyStates(){
    switch(feederCurrentState){
      case TRANSFERING_FUEL:
        setFeederVoltage(3);
        break;
      case STOPPED:
        stopFeeder();
        break;
      case REVERSE:
        setFeederVoltage(-3);
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

  public void setFeederWantedState(FeederWantedState feederWantedState){
    this.feederWantedState = feederWantedState;
  }

}