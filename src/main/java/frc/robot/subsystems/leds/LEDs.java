// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final LEDsIO io;
  private LEDWantedState wantedState = LEDWantedState.STOP;
  private LEDCurrentState currentState = LEDCurrentState.STOPPING;

  public enum LEDWantedState{
    STOP,
    CLIMB_EXTEND,
    CLIMB_RETRACT,
    AUTO_ALIGN,
    INTAKE_FUEL,
    SHOOT_FUEL,
    PULSE_FUEL,
  }

  public enum LEDCurrentState{
    STOPPING,
    CLIMB_EXTENDING,
    CLIMB_RETRACTING,
    AUTO_ALIGNING,
    INTAKING_FUEL,
    SHOOTING_FUEL,
    PULSING_FUEL,
  }

  public LEDs(LEDsIO io) {
    this.io = io;
  }

  public void updateInputs() {
    io.updateInputs();
    
    handleFeederStateTransitions();
    applyStates();

    DogLog.log("LEDs/WantedState", wantedState);
    DogLog.log("LEDs/CurrentState", currentState);

  }

  public void handleFeederStateTransitions(){
    switch (wantedState) {
        case STOP:
            currentState = LEDCurrentState.STOPPING;
            break;
        case CLIMB_EXTEND:
            currentState = LEDCurrentState.CLIMB_EXTENDING;
            break;
        case CLIMB_RETRACT:
            currentState = LEDCurrentState.CLIMB_RETRACTING;
            break;
        case INTAKE_FUEL:
            currentState = LEDCurrentState.INTAKING_FUEL;
            break;
        case SHOOT_FUEL:
            currentState = LEDCurrentState.SHOOTING_FUEL;
            break;
        case PULSE_FUEL:
            currentState = LEDCurrentState.PULSING_FUEL;
            break;
        default:
            currentState = LEDCurrentState.STOPPING;
            break;
    }
  }

  public void applyStates(){
    switch(currentState){
      case STOPPING:
        LEDsStop();
        break;
      case CLIMB_EXTENDING:
        LEDsClimbExtend();
        break;
      case CLIMB_RETRACTING:
        LEDsClimbRetract();
        break;
      case INTAKING_FUEL:
        LEDsIntakeFuel();
        break;
      case SHOOTING_FUEL:
        LEDsShootFuel();
        break;
      case PULSING_FUEL:
        LEDsPulseFuel();
        break;
      default:
        LEDsStop();
        break;
    }
  }

  public void LEDsStop(){
    io.LEDsStop();
  }

  public void LEDsClimbExtend(){
    io.LEDsClimbExtend();
  }

  public void LEDsClimbRetract(){
    io.LEDsClimbRetract();
  }

  public void LEDsAutoAlign(){
    io.LEDsAutoAlign();
  }

  public void LEDsIntakeFuel(){
    io.LEDsIntakeFuel();
  }

  public void LEDsShootFuel(){
    io.LEDsShootFuel();
  }

  public void LEDsPulseFuel(){
    io.LEDsPulseFuel();
  }

  public void setWantedState(LEDs.LEDWantedState wantedState){
    this.wantedState = wantedState;
  }

}