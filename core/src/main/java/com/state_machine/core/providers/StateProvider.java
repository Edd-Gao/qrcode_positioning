package com.state_machine.core.providers;

import com.state_machine.core.actions.Action;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.states.*;
import org.apache.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

public class StateProvider {

    private IdleState idleState;
    private ShutdownState shutdownState;
    private ManualControlState manualControlState;
    private RosServiceProvider serviceProvider;
    private ActionProvider actionProvider;
    private Log log;

    public StateProvider(ActionProvider actionProvider, RosServiceProvider serviceProvider, Log log){
        this.serviceProvider = serviceProvider;
        this.actionProvider = actionProvider;
        this.log = log;

        this.idleState = new IdleState(actionProvider, serviceProvider.getSetModeService(), log);

        this.shutdownState = new ShutdownState(actionProvider, serviceProvider.getSetModeService(), log);

        this.manualControlState = new ManualControlState(actionProvider, serviceProvider.getSetModeService(), log);
    }

    public IdleState getIdleState() { return idleState; }

    public ShutdownState getShutdownState() { return shutdownState; }

    public ManualControlState getManualControlState() { return manualControlState; }

    public ScriptedState getScriptedState(List<Action> actions) {
        return new ScriptedState(actions, actionProvider, serviceProvider.getSetModeService(), log);
    }

    public WaypointState getWaypointState(Queue<Waypoint> waypoints){
        return new WaypointState(waypoints, actionProvider, serviceProvider.getSetModeService(), log);
    }
}
