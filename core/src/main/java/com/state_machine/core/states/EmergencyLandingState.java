package com.state_machine.core.states;

import com.state_machine.core.providers.ActionProvider;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.message.Time;
import org.ros.node.service.ServiceClient;

/**
 * Created by parallels on 8/20/16.
 */
public class EmergencyLandingState extends State {

    public EmergencyLandingState(
            ActionProvider actionProvider,
            ServiceClient<SetModeRequest, SetModeResponse> setModeService,
            Log log
    ){
        super(actionProvider, setModeService, log);
        prerequisites.add(actionProvider.getSetFCUModeAction("STABILIZE"));
        prerequisites.add(actionProvider.getLandingAction());
        prerequisites.add(actionProvider.getDisarmAction());
    }

    public void chooseNextAction(Time time){
    }

    public boolean isIdling() { return true; }

    public boolean isSafeToExit(){
        return true;
    }

    public String toString() {
        return "IdleState";
    }

}
