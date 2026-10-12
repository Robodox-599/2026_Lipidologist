// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import dev.doglog.DogLog;

public class Indexer {
  private final IndexerIO io;
  private IndexerWantedState wantedState = IndexerWantedState.STOPPED;
  private IndexerCurrentState currentState = IndexerCurrentState.STOPPING;

  public enum IndexerWantedState {
    TRANSFER_FUEL,
    PULSE_FUEL,
    REVERSE,
    OUTAKE,
    CLEAN,
    STOPPED,
  }

  public enum IndexerCurrentState {
    TRANSFERING_FUEL,
    PULSING_FUEL,
    REVERSING,
    OUTAKING,
    CLEANING,
    STOPPING,
  }

  /** Creates a new indexer. */
  public Indexer(IndexerIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
  }

  public void updateStates() {
    handleStateTransitions();
    applyStates();

    DogLog.log("Indexer/wantedState", wantedState);
    DogLog.log("Indexer/currentState", currentState);
  }

  public void handleStateTransitions() {
    switch (wantedState) {
      case STOPPED:
        currentState = IndexerCurrentState.STOPPING;
        break;
      case TRANSFER_FUEL:
        currentState = IndexerCurrentState.TRANSFERING_FUEL;
        break;
      case PULSE_FUEL:
        currentState = IndexerCurrentState.PULSING_FUEL; // use time wpilib func
        break;
      case REVERSE:
        currentState = IndexerCurrentState.REVERSING;
        break;
      case OUTAKE:
        currentState = IndexerCurrentState.OUTAKING;
        break;
      case CLEAN:
        currentState = IndexerCurrentState.CLEANING;
        break;
      default:
        currentState = IndexerCurrentState.STOPPING;
        break;
    }
  }

  public void applyStates() {
    switch (currentState) {
      case STOPPING:
        stopIndexer();
        break;
      case TRANSFERING_FUEL:
        setIndexerDutyCycle(1.0);
        break;
      case PULSING_FUEL:
        indexerPulseFuel(1.0 / 12.0);
        break;
      case REVERSING:
        setIndexerDutyCycle(-1.0 / 12.0);
        break;
      case OUTAKING:
        setIndexerDutyCycle(-1.0);
        break;
      case CLEANING:
        setIndexerDutyCycle(0.65 / 12.0);
        break;
      default:
        stopIndexer();
        break;
    }
  }

  public void setIndexerDutyCycle(double dutyCycle) {
    io.setIndexerDutyCycle(dutyCycle);
  }

  public void stopIndexer() {
    io.stopIndexer();
  }

  public void setWantedState(Indexer.IndexerWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public void indexerPulseFuel(double dutyCycle) {
    io.indexerPulseFuel(dutyCycle);
  }
}
