// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import dev.doglog.DogLog;

public class Climb {
  private final ClimbIO io;
  public ClimbWantedState wantedState = ClimbWantedState.STOP;
  public ClimbCurrentState currentState = ClimbCurrentState.STOPPED;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  public enum ClimbWantedState {
    STOP,
    EXTENDING,
    RETRACTING,
    ZERO_CLIMB
  }

  public enum ClimbCurrentState {
    STOPPED,
    EXTENDING,
    RETRACTING,
    ZEROING_CLIMB
  }

  public void updateInputs() {
    io.updateInputs();
    handleStateTransitions();
    applyStates();

    DogLog.log("Climb/WantedState", wantedState);
    DogLog.log("Climb/CurrentState", currentState);

  }

  private void handleStateTransitions() {
    switch (wantedState) {
      case STOP:
        currentState = ClimbCurrentState.STOPPED;
        break;
      case EXTENDING:
        currentState = ClimbCurrentState.EXTENDING;
        break;
      case RETRACTING:
        currentState = ClimbCurrentState.RETRACTING;
        break;
      case ZERO_CLIMB:
        currentState = ClimbCurrentState.ZEROING_CLIMB;
    }
  }

  private void applyStates() {
    switch (currentState) {
      case STOPPED:
        stop();
        break;
      case EXTENDING:
        setPosition(0);
        break;
      case RETRACTING:
        setPosition(0);
        break;
      case ZEROING_CLIMB:
        zeroClimb();
    }
  }

  private void stop() {
    io.stop();
  }

  public boolean atSetpoint() {
    return io.atSetpoint;
  }

  private void setPosition(double position) {
    io.setPosition(position);
  }

  public void zeroClimb(){
    io.zeroClimb();
  }
}
