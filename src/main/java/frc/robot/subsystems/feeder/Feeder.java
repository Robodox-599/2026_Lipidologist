// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder {
  private final FeederIO io;
  private FeederWantedState wantedState = FeederWantedState.STOP;
  private FeederCurrentState currentState = FeederCurrentState.STOPPED;

  public Feeder(FeederIO io){
    this.io = io;
  }

  public enum FeederWantedState{
    STOP,
    FEED,
    REVERSE,
  }

  public enum FeederCurrentState{
    STOPPED,
    FEEDING,
    REVERSING
  }

  public void updateInputs(){
    io.updateInputs();
    handleStateTransitions();
    applyStates();
  }

  public void handleStateTransitions(){
    switch(wantedState){
      case STOP:
        currentState = FeederCurrentState.STOPPED;
        break;
      case FEED:
        currentState = FeederCurrentState.FEEDING;
        break;
      case REVERSE:
        currentState = FeederCurrentState.REVERSING;
        break;
      default:
        currentState = FeederCurrentState.STOPPED;
        break;
    }
  }

  public void applyStates(){
    switch(currentState){
      case STOPPED:
        stop();
        break;
      case FEEDING:
        setVoltage(0);
        break;
      case REVERSING:
        setVoltage(0);
        break;
    }
  }

  private void stop(){
    io.stop();
  }

  private void setVoltage(double voltage){
    io.setVoltage(voltage);
  }
}