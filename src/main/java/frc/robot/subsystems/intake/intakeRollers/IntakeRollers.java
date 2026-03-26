package frc.robot.subsystems.intake.intakeRollers; // make sure the folder name is all lowercase

import dev.doglog.DogLog;
import frc.robot.util.Tracer;

public class IntakeRollers {
    private final IntakeRollersIO io;
    private IntakeRollersWantedState wantedState = IntakeRollersWantedState.STOP;
    private IntakeRollersCurrentState currentState = IntakeRollersCurrentState.STOPPED;
    
    public IntakeRollers(IntakeRollersIO io){
        this.io = io;
    }

    public enum IntakeRollersWantedState{
        STOP,
        INTAKE_FUEL,
        AGITATE_FUEL,
        OUTAKE
    }

    public enum IntakeRollersCurrentState{
        STOPPED,
        INTAKING_FUEL,
        AGITATING_FUEL,
        OUTAKING
    }

    public void updateInputs(){
        Tracer.traceFunc("IntakeRollers UpdateInputs", io::updateInputs);
        handleStateTransitions();
        // applyStates();
        DogLog.log("Intake/Rollers/WantedState", this.wantedState);
        DogLog.log("Intake/Rollers/CurrentState", this.currentState);
    }

    private void handleStateTransitions(){
        switch(wantedState){
            case STOP:
                currentState = IntakeRollersCurrentState.STOPPED;
                break;
            case INTAKE_FUEL:
                currentState = IntakeRollersCurrentState.INTAKING_FUEL;
                break;
            case AGITATE_FUEL:
                currentState = IntakeRollersCurrentState.AGITATING_FUEL;
                break;
            case OUTAKE:
                currentState = IntakeRollersCurrentState.OUTAKING;
                break;
            default:
                currentState = IntakeRollersCurrentState.STOPPED;
                break;
        }
    } 

    private void applyStates(){
        switch(currentState){
            case STOPPED:
                stop();
                break;
            case INTAKING_FUEL:
                setVoltage(7);
                break;
            case AGITATING_FUEL:
                setVoltage(2);
                break;
            case OUTAKING:
                setVoltage(-7);
                break;
            default:
                stop();
                break;
        }
    }

    public void stop(){
        io.stop();
    }

    public void setWantedState(IntakeRollers.IntakeRollersWantedState wantedState){
        this.wantedState = wantedState;
    }

    public void setVoltage(double voltage){
        io.setVoltage(voltage);
    }
}


