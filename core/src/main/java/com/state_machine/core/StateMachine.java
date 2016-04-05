package com.state_machine.core;

public class StateMachine {

    private State state;
    private State failsafeState;

    public StateMachine(State initialState, State failsafeState){
        this.failsafeState = failsafeState;
        state = initialState;
        initialState.entryAction();
    }

    public State getState(){
        return state;
    }

    public void update(){
        ErrorType error = failsafeCheck();
        switch (error) {
            case NoError:
                state.loopAction();
                break;
            case BatteryLow:
            case ConnectionError:
                setState(failsafeState);
                break;
        }
    }

    public ErrorType failsafeCheck(){
        return ErrorType.NoError;
    }

    public void setState(State newState){
        state.exitAction();
        state = newState;
        newState.entryAction();
    }
}
