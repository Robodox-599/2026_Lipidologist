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
    EXTEND,
    RETRACT,
    ZERO_CLIMB,
    STOW
  }

  public enum ClimbCurrentState {
    STOPPED,
    EXTENDING,
    RETRACTING,
    ZEROING_CLIMB,
    STOWED
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
      case EXTEND:
        currentState = ClimbCurrentState.EXTENDING;
        break;
      case RETRACT:
        if (atSetpoint(0)) {
          currentState = ClimbCurrentState.STOWED;
        } else {
          currentState = ClimbCurrentState.RETRACTING;
        }
        break;
      case ZERO_CLIMB:
        currentState = ClimbCurrentState.ZEROING_CLIMB;
        break;
      case STOW:
        currentState = ClimbCurrentState.STOWED;
        break;
      default:
        currentState = ClimbCurrentState.STOPPED;
        break;
    }
  }

  public void applyStates() {
    switch (currentState) {
      case STOPPED:
        stop();
        break;
      case EXTENDING:
        setPosition(ClimbConstants.climbExtensionPosition);
        break;
      case RETRACTING:
        setPosition(0);
        break;
      case ZEROING_CLIMB:
        zeroClimb();
        break;
      case STOWED:
        setPosition(0);
        break;
      default:
        stop();
    }
  }

  private void stop() {
    io.stop();
  }

  public boolean atSetpoint() {
    return io.atSetpoint;
  }

  public boolean atSetpoint(double targetPosition) {
    return Math.abs(targetPosition - io.position) < 0.02;
  }
  
  public boolean atExtensionSetpoint() {
    return Math.abs(ClimbConstants.climbExtensionPosition - io.position) < 0.02;
  }

  public boolean atStowSetpoint() {
    return Math.abs(io.position) < 0.02;
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void zeroClimb() {
    io.zeroClimb();
  }

  public void setWantedState(Climb.ClimbWantedState wantedState) {
    this.wantedState = wantedState;
  }
}
