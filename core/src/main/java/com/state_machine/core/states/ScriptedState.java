package com.state_machine.core.states;

import com.state_machine.core.actions.Action;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.util.List;

public class ScriptedState extends State {

    public ScriptedState(List<Action> scriptedActions,
                         ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                         Log log){
        super(scriptedActions, setModeService, log);
    }

    public void chooseNextAction(Time time){
        currentAction = prerequisites.get(prerequisites.size() - 1);
    }

    public boolean isSafeToExit(){ return true; }

    @Override public void enterAction(){
        SetModeRequest request = setModeService.newMessage();
        request.setCustomMode("OFFBOARD");
        setModeService.call(request, new ServiceResponseListener<SetModeResponse>() {
            @Override
            public void onSuccess(SetModeResponse setModeResponse) {

            }

            @Override
            public void onFailure(RemoteException e) {
                log.warn("Failed to set mode", e);
                lastFailure = new Failure(ErrorType.ConnectionFailure, currentTime);
            }
        });
        super.enterAction();
    }

    public String toString() {
        return "ScriptedState";
    }
}
