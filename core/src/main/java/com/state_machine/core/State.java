package com.state_machine.core;

public abstract class State {

    public abstract void entryAction();

    public abstract void loopAction();

    public abstract void exitAction();

    public void update(){
        failsafeCheck();
        loopAction();
    }

    public boolean failsafeCheck(){
        return false;
    }
}
