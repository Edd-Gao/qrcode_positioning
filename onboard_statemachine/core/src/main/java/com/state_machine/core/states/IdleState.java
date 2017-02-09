package com.state_machine.core.states;

import com.state_machine.core.providers.ActionProvider;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.node.service.ServiceClient;

public class IdleState extends State {

    private long startTime;
    private long duration;
    public IdleState(ActionProvider actionProvider,
                     ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                     Log log, long duration) {
        super(actionProvider, setModeService, log);
        this.duration = duration;
        currentAction = null;
        startTime = System.currentTimeMillis();
    }

    public boolean isSafeToExit(){
        if((System.currentTimeMillis() - startTime) > duration)
            return true;
        else
            return false;
    }

    public String toString() {
        return "IdleState";
    }
}
