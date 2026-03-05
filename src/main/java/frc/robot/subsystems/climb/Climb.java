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


  public void handleStateTransitions() {
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


  public void applyStates() {
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


  public void setPosition(double position) {
    io.setPosition(position);
  }


  public void zeroClimb(){
    io.zeroClimb();
  }

    public void setWantedState(Climb.ClimbWantedState wantedState) {
    this.wantedState = wantedState;
  }
}


