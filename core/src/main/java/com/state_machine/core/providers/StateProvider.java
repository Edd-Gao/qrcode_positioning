package com.state_machine.core.providers;

import com.state_machine.core.actions.Action;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.states.*;
import org.apache.commons.logging.Log;

import java.util.List;
import java.util.Queue;

public class StateProvider {

    private IdleState idleState;
    private EmergencyLandingState emergencyLandingState;
    private RosServiceProvider serviceProvider;
    private RosPublisherProvider rosPublisherProvider;
    private ActionProvider actionProvider;
    private Log log;

    public StateProvider(ActionProvider actionProvider, RosServiceProvider serviceProvider, RosPublisherProvider rosPublisherProvider, Log log){
        this.serviceProvider = serviceProvider;
        this.actionProvider = actionProvider;
        this.rosPublisherProvider = rosPublisherProvider;
        this.log = log;

        this.idleState = new IdleState(actionProvider, serviceProvider.getSetFCUModeService(), log);
        this.emergencyLandingState = new EmergencyLandingState(actionProvider, serviceProvider.getSetFCUModeService(), log);

    }

    public IdleState getIdleState() { return idleState; }

    public EmergencyLandingState getEmergencyLandingState(){ return emergencyLandingState;}

    public ScriptedState getScriptedState(Queue<Action> actions) {
        return new ScriptedState(actions, actionProvider, serviceProvider.getSetFCUModeService(), log);
    }

}
