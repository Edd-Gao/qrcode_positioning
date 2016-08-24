package com.state_machine.core.states;

import com.state_machine.core.actions.Action;
import com.state_machine.core.providers.ActionProvider;
import com.state_machine.core.states.util.ErrorType;
import com.state_machine.core.states.util.Failure;
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
                         ActionProvider actionProvider,
                         ServiceClient<SetModeRequest, SetModeResponse> setModeService,
                         Log log){
        super(actionProvider, setModeService, log);
        prerequisites.add(actionProvider.getSetFCUModeAction("STABILIZE"));
        prerequisites.addAll(scriptedActions);
    }

    public void chooseNextAction(Time time){}

    public boolean isSafeToExit(){ return true; }

    public boolean isIdling(){ return false;}

    public String toString() {
        return "ScriptedState";
    }
}
