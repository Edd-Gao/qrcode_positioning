package com.state_machine.core.states;

import com.state_machine.core.actions.Action;
import com.state_machine.core.providers.ActionProvider;
import mavros_msgs.SetModeRequest;
import mavros_msgs.SetModeResponse;
import org.apache.commons.logging.Log;
import org.ros.node.service.ServiceClient;

import java.util.Queue;

public class ScriptedState extends State {


    public ScriptedState(Queue<Action> scriptedActions,
                         ActionProvider actionProvider,
                         ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                         Log log){
        super(actionProvider, setModeService, log);
        //actionQueue.add(actionProvider.getSetFCUModeAction("OFFBOARD"));
        actionQueue.addAll(scriptedActions);
    }

    public boolean isSafeToExit(){
        if (currentAction == null)
            return true;
        else
            return false;
    }


    public String toString() {
        return "ScriptedState";
    }
}
