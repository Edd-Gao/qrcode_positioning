package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import org.ros.message.Time;

public abstract class Action {

    protected ActionStatus status;

    public abstract ActionStatus loopAction(Time time);

    public Action() {
        status = ActionStatus.Inactive;
    }

    public void enterAction() { status = ActionStatus.Inactive; }

    public void exitAction(){}
}
