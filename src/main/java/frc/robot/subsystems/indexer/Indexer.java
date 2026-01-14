// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private IndexerWantedState indexerWantedState = IndexerWantedState.STOPPED;
  private IndexerCurrentState indexerCurrentState = IndexerCurrentState.STOPPING;

  public enum IndexerWantedState{
    TRANSFER_FUEL,
    PULSE_FUEL,
    REVERSE,
    STOPPED,
  }

  public enum IndexerCurrentState{
    TRANSFERING_FUEL,
    PULSING_FUEL,
    REVERSING,
    STOPPING,
  }

  /** Creates a new indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void updateIndexerInputs() {
    io.updateIndexerInputs();
    
    handleStateTransitions();
    applyStates();

    DogLog.log("Indexer/wantedState", indexerWantedState);
    DogLog.log("Indexer/currentState", indexerCurrentState);

  }

  public void handleStateTransitions(){
    switch (indexerCurrentState) {
        case STOPPING:
            indexerCurrentState = IndexerCurrentState.STOPPING;
            break;
        case TRANSFERING_FUEL:
            indexerCurrentState = IndexerCurrentState.TRANSFERING_FUEL;
            break;
        case PULSING_FUEL:
            indexerCurrentState = IndexerCurrentState.PULSING_FUEL; // use time wpilib func
            break;
        case REVERSING:
            indexerCurrentState = IndexerCurrentState.REVERSING;
            break;
        default:
            indexerCurrentState = IndexerCurrentState.STOPPING;
            break;
    }
  }

  public void applyStates(){
    switch(indexerCurrentState){
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

  public void setIndexerWantedState(IndexerWantedState indexerWantedState){
    this.indexerWantedState = indexerWantedState;
  }

  public void indexerPulseFuel(double volts){
    io.indexerPulseFuel(volts);
  }

}