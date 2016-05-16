package com.state_machine.core.states;

import com.state_machine.core.actions.Action;
import com.state_machine.core.providers.ActionProvider;
import com.state_machine.core.states.util.ErrorType;
import com.state_machine.core.states.util.Failure;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.lang.NotImplementedException;
import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.util.List;

public class ShutdownState extends State {

    public ShutdownState(ActionProvider actionProvider,
                         ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                         Log log) {
        super(actionProvider, setModeService, log);
        prerequisites.add(actionProvider.getLandingAction());
        prerequisites.add(actionProvider.getDisarmAction());
    }

    public void chooseNextAction(Time time){
        throw new NotImplementedException();
    }

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

    public boolean isIdling() { return false; }

    public boolean isSafeToExit(){
        return true;
    }

    public String toString() {
        return "ShutdownState";
    }
}
