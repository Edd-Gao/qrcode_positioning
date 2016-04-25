package com.state_machine.core.states;

import com.state_machine.core.actions.Action;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.lang.NotImplementedException;
import org.apache.commons.logging.Log;
import org.ros.exception.RemoteException;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import java.util.List;

public class IdleState extends State {

    public IdleState(List<Action> prerequisites,
                     ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                     Log log) {
        super(prerequisites, setModeService, log);
    }

    public void chooseNextAction(Time time){
        throw new NotImplementedException();
    }

    @Override public void enterAction(){
        SetModeRequest request = setModeService.newMessage();
        request.setCustomMode("STABILIZED");
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

    public boolean isSafeToExit(){
        return true;
    }

    public String toString() {
        return "IdleState";
    }
}
