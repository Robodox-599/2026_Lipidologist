// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

public class Indexer extends SubsystemBase {
  IndexerIO io;
  private IndexerWantedState wantedState = IndexerWantedState.STOP;
  private IndexerCurrentState currentState = IndexerCurrentState.STOPPED;
  private Timer pulseTimer = new Timer();

  public enum IndexerWantedState{
    STOP,
    INDEX,
    REVERSE,
    PULSE,

  }

  public enum IndexerCurrentState{
    STOPPED,
    INDEXING,
    REVERSING,
  }

  public Indexer(IndexerIO io){
    this.io = io;
    pulseTimer.start();
  }

  public void updateInputs(){
    io.updateInputs();
    handleStateTransitions();
    applyStates();

    DogLog.log("Indexer/WantedState", wantedState);
    DogLog.log("Indexer/CurrentState", currentState);
  }

  private void handleStateTransitions(){
    switch(wantedState){
      case STOP:
        currentState = IndexerCurrentState.STOPPED;
        break;
      case INDEX:
        currentState = IndexerCurrentState.INDEXING;
        break;
      case REVERSE:
        currentState = IndexerCurrentState.REVERSING;
        break;
      case PULSE:
        if (pulseTimer.get() >= IndexerConstants.pulseTime){
          if (currentState == IndexerCurrentState.REVERSING){
            currentState = IndexerCurrentState.INDEXING;
            pulseTimer.reset();
          } else if (currentState == IndexerCurrentState.INDEXING){
            currentState = IndexerCurrentState.REVERSING;
            pulseTimer.reset();
          }
        } else {
          if (currentState == IndexerCurrentState.INDEXING){
            currentState = IndexerCurrentState.INDEXING;
          } else {
            currentState = IndexerCurrentState.REVERSING;
          }
        }
        break;
      default:
        currentState = IndexerCurrentState.STOPPED;
        break;

    }
  }
  
  private void applyStates(){
    switch(currentState){
      case STOPPED:
        stop();
        break;
      case INDEXING:
        setVoltage(0);
        break;
      case REVERSING:
        setVoltage(0);
        break;
      default:
        stop();
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