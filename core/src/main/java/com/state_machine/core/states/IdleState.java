package com.state_machine.core.states;

import com.state_machine.core.providers.ActionProvider;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.node.service.ServiceClient;

public class IdleState extends State {

    public IdleState(ActionProvider actionProvider,
                     ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                     Log log) {
        super(actionProvider, setModeService, log);
        actionQueue.add(actionProvider.getSetFCUModeAction("STABILIZE"));
        actionQueue.add(actionProvider.getDisarmAction());
    }

    public boolean isSafeToExit(){
        return true;
    }

    public String toString() {
        return "IdleState";
    }
}
