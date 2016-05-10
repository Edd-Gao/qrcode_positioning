package com.state_machine.core.providers;

import com.state_machine.core.actions.Action;
import com.state_machine.core.script.FlightScriptParser;
import com.state_machine.core.states.IdleState;
import com.state_machine.core.states.ManualControlState;
import com.state_machine.core.states.ScriptedState;
import com.state_machine.core.states.ShutdownState;
import org.apache.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

public class StateProvider {

    private IdleState idleState;
    private ShutdownState shutdownState;
    private ManualControlState manualControlState;
    private RosServiceProvider serviceProvider;
    private Log log;

    public StateProvider(ActionProvider actionProvider, RosServiceProvider serviceProvider, Log log){
        this.serviceProvider = serviceProvider;
        this.log = log;

        List<Action> prereqs = new ArrayList<>();
        prereqs.add(actionProvider.getArmAction());
        this.idleState = new IdleState(prereqs, serviceProvider.getSetModeService(), log);

        prereqs = new ArrayList<>();
        prereqs.add(actionProvider.getLandingAction());
        prereqs.add(actionProvider.getDisarmAction());
        this.shutdownState = new ShutdownState(prereqs, serviceProvider.getSetModeService(), log);

        prereqs = new ArrayList<>();
        this.manualControlState = new ManualControlState(prereqs, serviceProvider.getSetModeService(), log);
    }

    public IdleState getIdleState() { return idleState; }

    public ShutdownState getShutdownState() { return shutdownState; }

    public ManualControlState getManualControlState() { return manualControlState; }

    public ScriptedState getScriptedState(List<Action> actions) {
        return new ScriptedState(actions, serviceProvider.getSetModeService(), log);
    }
}
