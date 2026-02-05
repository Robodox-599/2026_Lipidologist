// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private WantedState wantedState = WantedState.STOPPED;
  private CurrentState currentState = CurrentState.STOPPING;

  public enum WantedState{
    TRANSFER_FUEL,
    PULSE_FUEL,
    REVERSE,
    STOPPED,
  }

  public enum CurrentState{
    TRANSFERING_FUEL,
    PULSING_FUEL,
    REVERSING,
    STOPPING,
  }

  /** Creates a new indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
    
    handleStateTransitions();
    applyStates();

    DogLog.log("Indexer/wantedState", wantedState);
    DogLog.log("Indexer/currentState", currentState);

  }

  public void handleStateTransitions(){
    switch (wantedState) {
        case STOPPED:
            currentState = CurrentState.STOPPING;
            break;
        case TRANSFER_FUEL:
            currentState = CurrentState.TRANSFERING_FUEL;
            break;
        case PULSE_FUEL:
            currentState = CurrentState.PULSING_FUEL; // use time wpilib func
            break;
        case REVERSE:
            currentState = CurrentState.REVERSING;
            break;
        default:
            currentState = CurrentState.STOPPING;
            break;
    }
  }

  public void applyStates(){
    switch(currentState){
      case STOPPING:
        stopIndexer();
        break;
      case TRANSFERING_FUEL:
        setIndexerVoltage(3);
        break;
      case PULSING_FUEL:
        indexerPulseFuel(1);;
        break;
      case REVERSING:
        setIndexerVoltage(-3);
        break;
      default:
        stopIndexer();
        break;
    }
  }

  public void setIndexerVoltage(double volts){
    io.setIndexerVoltage(volts);
  }

  public void stopIndexer(){
    io.stopIndexer();
  }

  public void setWantedState(Indexer.WantedState wantedState){
    this.wantedState = wantedState;
  }

  public void indexerPulseFuel(double volts){
    io.indexerPulseFuel(volts);
  }

}