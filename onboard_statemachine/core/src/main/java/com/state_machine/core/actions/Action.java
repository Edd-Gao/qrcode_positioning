package com.state_machine.core.actions;

import com.state_machine.core.actions.util.ActionStatus;
import org.apache.commons.logging.Log;
import org.ros.message.Duration;
import org.ros.message.Time;
import org.ros.time.TimeProvider;

public abstract class Action {

    // timeStamp records the start time of an action.
    protected Time timeStamp = new Time(0,0);

    protected ActionStatus status;

    protected static final Duration enterTimeOut = new Duration(0,50000);

    public abstract ActionStatus loopAction(Time time);

    public Action() {
        status = ActionStatus.Inactive;
    }

    public ActionStatus enterAction(Time time) { status = ActionStatus.Inactive; return status;}

    public void exitAction(){}
}
