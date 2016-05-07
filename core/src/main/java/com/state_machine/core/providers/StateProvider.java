package com.state_machine.core.providers;

import com.state_machine.core.actions.Action;
import com.state_machine.core.actions.ArmAction;
import com.state_machine.core.script.FlightScript;
import com.state_machine.core.states.IdleState;
import com.state_machine.core.states.ManualControlState;
import com.state_machine.core.states.ScriptedState;
import com.state_machine.core.states.ShutdownState;
import org.apache.commons.logging.Log;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class StateProvider {

    private IdleState idleState;
    private ShutdownState shutdownState;
    private ManualControlState manualControlState;
    private ScriptedState scriptedState;

    public StateProvider(ActionProvider actionProvider, RosServiceProvider serviceProvider, Log log){
        List<Action> prereqs = new ArrayList<>();
        prereqs.add(actionProvider.getArmAction());
        this.idleState = new IdleState(prereqs, serviceProvider.getSetModeService(), log);

        prereqs = new ArrayList<>();
        prereqs.add(actionProvider.getLandingAction());
        prereqs.add(actionProvider.getDisarmAction());
        this.shutdownState = new ShutdownState(prereqs, serviceProvider.getSetModeService(), log);

        prereqs = new ArrayList<>();
        this.manualControlState = new ManualControlState(prereqs, serviceProvider.getSetModeService(), log);

        FlightScript script = new FlightScript("flightscript.json", actionProvider, log);
        scriptedState = new ScriptedState(script.getActions(), serviceProvider.getSetModeService(), log);
    }

    public IdleState getIdleState() { return idleState; }

    public ShutdownState getShutdownState() { return shutdownState; }

    public ManualControlState getManualControlState() { return manualControlState; }

    public ScriptedState getScriptedState() { return scriptedState; }
}
