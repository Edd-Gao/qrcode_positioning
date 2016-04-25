package com.state_machine.core.actions;

import org.ros.message.Time;

public abstract class Action {

    protected ActionStatus status;

    public abstract ActionStatus loopAction(Time time);

    public Action() {
        status = ActionStatus.Inactive;
    }

    public void enterAction() {}

    public void exitAction(){
        status = ActionStatus.Inactive;
    }
}
