package com.state_machine.core.states;

import com.state_machine.core.actions.Action;
import com.state_machine.core.actions.util.ActionStatus;
import com.state_machine.core.states.util.ErrorType;
import com.state_machine.core.states.util.Failure;
import com.state_machine.core.states.util.StateHandle;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;

import java.util.List;

public abstract class State implements StateHandle {

    protected List<Action> prerequisites;
    protected ServiceClient<SetModeRequest, SetModeResponse> setModeService;
    protected Action currentAction;
    protected Time currentTime;
    protected Failure lastFailure;
    protected Log log;

    public State(List<Action> prerequisites,
                 ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                 Log log){
        this.prerequisites = prerequisites;
        this.setModeService = setModeService;
        this.log = log;
    }

    final public void update(Time time) {
        currentTime = time;
        if(currentAction == null) {
            chooseNextAction(time);
        }

        if(currentAction != null) {
            ActionStatus status = currentAction.loopAction(time);
            while(handleActionResult(status) && currentAction != null){
                currentAction.loopAction(time);
            }
        }
    }

    public void enterAction(){
        lastFailure = null;
        currentAction = null;
        if (prerequisites.size() > 0)
            setAction(prerequisites.get(0));
    }

    protected abstract void chooseNextAction(Time time);

    public void exitAction(){
        if (currentAction != null) {
            currentAction.exitAction();
        }
    }

    public abstract boolean isSafeToExit();

    public abstract boolean isIdling();

    public boolean isConnected(){
        return setModeService.isConnected();
    }

    public Failure getLastFailure(){ return lastFailure; }

    public Action getCurrentAction(){ return currentAction; }

    private boolean handleActionResult(ActionStatus status) {
        switch (status) {
            case Success:
                int index = prerequisites.indexOf(currentAction);
                if (index > 0 && index < prerequisites.size() - 1) {
                    currentAction = prerequisites.get(index + 1);
                }
                else {
                    currentAction = null;
                }
                return true;
            case ConnectionFailure:
                lastFailure = new Failure(ErrorType.ConnectionFailure, currentTime);
            case Failure:
            case Inactive:
                lastFailure = new Failure(ErrorType.ActionFailure, currentTime);
            case Waiting:
            default:
                return false;
        }
    }

    protected void setAction(Action newAction) {
        if (currentAction != null) {
            currentAction.exitAction();
        }
        currentAction = newAction;
        newAction.enterAction();
    }
}
