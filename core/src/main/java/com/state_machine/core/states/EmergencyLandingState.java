package com.state_machine.core.states;

import com.state_machine.core.droneState.DroneLanded;
import com.state_machine.core.droneState.DroneStateTracker;
import com.state_machine.core.providers.ActionProvider;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.node.service.ServiceClient;

/**
 * Created by parallels on 8/20/16.
 */
public class EmergencyLandingState extends State {

    private DroneStateTracker stateTracker;

    public EmergencyLandingState(
            ActionProvider actionProvider,
            ServiceClient<SetModeRequest, SetModeResponse> setModeService,
            Log log,
            DroneStateTracker stateTracker
    ){
        super(actionProvider, setModeService, log);
        this.stateTracker = stateTracker;
        actionQueue.add(actionProvider.getSetFCUModeAction("OFFBOARD"));
        actionQueue.add(actionProvider.getLandingAction());
        actionQueue.add(actionProvider.getDisarmAction());
    }

    public boolean isSafeToExit(){
        if(stateTracker.getDroneLanded()== DroneLanded.OnGround && !stateTracker.getArmed())
            return true;
        else
            return false;
    }

    public String toString() {
        return "EmergencyLandingState";
    }

}
