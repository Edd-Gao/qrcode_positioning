package com.state_machine.core;

public class StateMachine {

    private State state;

    public StateMachine(){
        state = null;
    }

    public State getState(){
        return state;
    }

    public void update(){
        state.update();
    }

    public void setState(State newState){
        state.exitAction();
        state = newState;
        newState.entryAction();
    }
}
