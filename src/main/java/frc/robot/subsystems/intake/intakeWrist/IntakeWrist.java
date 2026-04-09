// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake.intakeWrist;

import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.Tracer;

/** Add your docs here. */
public class IntakeWrist {
    private final IntakeWristIO io;
    private IntakeWristWantedState wantedState = IntakeWristWantedState.STOP;
    private IntakeWristCurrentState currentState = IntakeWristCurrentState.STOPPED;
    private Timer agitationTimer = new Timer();

    public IntakeWrist(IntakeWristIO io) {
        this.io = io;
        agitationTimer.start();
    }

    public enum IntakeWristWantedState {
        STOP,
        INTAKE_FUEL,
        LIFT,
        STOW, // pack
        AGITATE_FUEL,
    }

    public enum IntakeWristCurrentState {
        STOPPED,
        INTAKING_FUEL,
        STOWING, // packing
        LIFTING,
        WRIST_RETRACTING,
        WRIST_EXTENDING,
        UNJAM
    }

    public void updateInputs() {
        Tracer.traceFunc("IntakeWrist UpdateInputs", io::updateInputs);

        handleStateTransitions();
        applyStates();   

        DogLog.log("Intake/Wrist/WantedState", wantedState);
        DogLog.log("Intake/Wrist/CurrentState", currentState);
        DogLog.log("Intake/Wrist/IsWristJammed", io.isWristJammed);
        DogLog.log("Intake/Wrist/AgitationTimer", agitationTimer.get());
    }

    public void handleStateTransitions() {
        switch (wantedState) {
            case STOP:
                currentState = IntakeWristCurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                if ((io.isWristJammed && DriverStation.isAutonomous())) { // if in auto and wrist is jammed, unjam
                                                                          // hopper
                    currentState = IntakeWristCurrentState.UNJAM;
                } else {
                    currentState = IntakeWristCurrentState.INTAKING_FUEL;
                }
                break;
            case LIFT:
                currentState = IntakeWristCurrentState.LIFTING;
                break;
            case STOW:
                currentState = IntakeWristCurrentState.STOWING;
                break;
            case AGITATE_FUEL:
                // if (currentState == IntakeWristCurrentState.WRIST_RETRACTING){
                // if (atSetpoint() || io.isWristJammed){
                // currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                // } else{
                // currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                // }
                // } else if(currentState == IntakeWristCurrentState.WRIST_EXTENDING){
                // if (atSetpoint() || io.isWristJammed){
                // currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                // } else{
                // currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                // }
                // } else {
                // currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                // }

                if (agitationTimer.get() > IntakeWristConstants.agitationTime) {
                    if (currentState == IntakeWristCurrentState.WRIST_RETRACTING) {
                        currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                        agitationTimer.restart();
                    } else {
                        currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                        agitationTimer.restart();
                    }
                } else if (currentState == IntakeWristCurrentState.WRIST_RETRACTING) {
                    currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                } else {
                    currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                }
                break;
            default:
                currentState = IntakeWristCurrentState.STOPPED;
                break;
        }
    }

    public void applyStates() {
        switch (currentState) {
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setPosition(0.005);
                break;
            case LIFTING:
                setPosition(0.1);
                break;
            case STOWING:
                setPosition(.35);
                break;
            case WRIST_RETRACTING:
                setPosition(0.1);
                break;
            case WRIST_EXTENDING:
                setPosition(0.005);
                break;
            case UNJAM:
                setPosition(0.33);
                break;
            default:
                stop();
                break;

        }
    }

    public void stop() {
        io.stop();
    }

    public void setWantedState(IntakeWrist.IntakeWristWantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setPosition(double position) {
        io.setPosition(position);
    }

    public boolean atSetpoint() {
        return io.atSetpoint;
    }
}
