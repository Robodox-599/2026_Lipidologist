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
    private Debouncer wristStallDebouncer = new Debouncer(0.5, DebounceType.kBoth);
    private boolean isWristJammed = false;

    public IntakeWrist(IntakeWristIO io){
        this.io = io;
        agitationTimer.start();
    }

    public enum IntakeWristWantedState{
        STOP,
        INTAKE_FUEL,
        STOW, //pack
        AGITATE_FUEL,
    }

    public enum IntakeWristCurrentState{
        STOPPED,
        INTAKING_FUEL,
        STOWING, //packing
        WRIST_RETRACTING,
        WRIST_EXTENDING,
        UNJAM
    }

    public void updateInputs(){
        Tracer.traceFunc("IntakeWrist UpdateInputs", io::updateInputs);
        
        this.isWristJammed = wristStallDebouncer.calculate(io.statorCurrent > 60);

        handleStateTransitions();
        applyStates();

        DogLog.log("Intake/Wrist/WantedState", wantedState);
        DogLog.log("Intake/Wrist/CurrentState", currentState);
        DogLog.log("Intake/Wrist/IsWristJammed", isWristJammed);
    }

    public void handleStateTransitions(){
        switch(wantedState){
            case STOP:
                currentState = IntakeWristCurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                if (this.isWristJammed && DriverStation.isAutonomous()) {
                    currentState = IntakeWristCurrentState.UNJAM;
                } else {
                    currentState = IntakeWristCurrentState.INTAKING_FUEL;
                }
                break;
            case STOW:
                currentState = IntakeWristCurrentState.STOWING;
                break;
            case AGITATE_FUEL:
                // if (currentState == IntakeWristCurrentState.WRIST_RETRACTING){
                //     if (atSetpoint()){
                //         currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                //     } else{
                //         currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                //     }
                // } else if(currentState == IntakeWristCurrentState.WRIST_EXTENDING){
                //     if (atSetpoint()){
                //         currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                //     } else{
                //         currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                //     } 
                // } else {
                //     currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                // }

                if (agitationTimer.get() > IntakeWristConstants.agitationTime){
                    if (currentState == IntakeWristCurrentState.WRIST_RETRACTING){
                        currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                        agitationTimer.restart();
                    } else {
                        currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                        agitationTimer.restart();
                    }
                } else if (currentState == IntakeWristCurrentState.WRIST_RETRACTING){
                    currentState = IntakeWristCurrentState.WRIST_RETRACTING;
                } else{
                    currentState = IntakeWristCurrentState.WRIST_EXTENDING;
                }
                break;
            default:
                currentState = IntakeWristCurrentState.STOPPED;
                break;
        }
    }

    public void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setPosition(0.003);
                break;
            case STOWING:
                setPosition(.33);
                break;
            case WRIST_RETRACTING:
                setPosition(0.1);
                break;
            case WRIST_EXTENDING:
                setPosition(0.003);
                break;
            case UNJAM:
                setPosition(0.3);
                break;
            default:
                stop();
                break;

        }
    }

    public void stop(){
        io.stop();
    }

    public void setWantedState(IntakeWrist.IntakeWristWantedState wantedState){
        this.wantedState = wantedState;
    }

    public void setPosition(double position){
        io.setPosition(position);
    }

    public boolean atSetpoint(){
        return io.atSetpoint;
    }
}
