package com.state_machine.core.io;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.state_machine.core.actions.Action;
import com.state_machine.core.actions.util.Waypoint;
import com.state_machine.core.providers.ActionProvider;
import com.state_machine.core.providers.StateProvider;
import com.state_machine.core.states.State;
import org.apache.commons.logging.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.*;

public class FlightScriptParser {

    private StateProvider stateProvider;
    private ActionProvider actionProvider;
    private Log log;
    private Gson gson;
    private int seq;//waypoint sequence

    public FlightScriptParser(ActionProvider actions, StateProvider states, Log log){
        this.stateProvider = states;
        this.actionProvider = actions;
        this.gson = new GsonBuilder().create();
        this.log = log;
        seq = 0;
    }

    public Queue<State> parseFile(String filePath){
        Queue<State> states = new ArrayDeque<>();

        try {
            InputStream in = new FileInputStream(new File(filePath));
            Scanner scanner = null;
            StringBuilder json = new StringBuilder();
            List<StateJsonRepresentation> stateInfo = new ArrayList<>();
            try{
                scanner = new Scanner(in);
                while(scanner.hasNextLine()){
                    json.append(scanner.nextLine() + "\n");
                }
                stateInfo = gson.fromJson(json.toString(), JsonStateList.class).queue;
            } catch (Exception e){
                log.warn("Could not read flight io at " + filePath, e);
            } finally {
                if(scanner != null) scanner.close();
            }
            for(StateJsonRepresentation s : stateInfo){
                State state = parseState(s);
                if(state != null) states.add(state);
            }
            return states;
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return states;
        }
    }

    private State parseState(StateJsonRepresentation repr){
        switch(repr.state){
            case "IdleState":
                return stateProvider.getIdleState(repr.duration);
            case "ScriptedState":
                Queue<Action> actions = new ArrayDeque<>();
                for(ActionJsonRepresentation a : repr.scriptedActions){
                    Action action = parseAction(a);
                    if(action != null) actions.add(action);
                }
                return stateProvider.getScriptedState(actions);
            default:
                log.warn("Flight io contained invalid state: " + repr.state);
                return null;
        }
    }

    private Action parseAction(ActionJsonRepresentation repr){
        switch(repr.action){
            case "ArmAction":
                return actionProvider.getArmAction();
            case "DisarmAction":
                return actionProvider.getDisarmAction();
            case "TakeOffAction":
                return actionProvider.getTakeoffAction(repr.target_heightcm);
            case "LandingAction":
                return actionProvider.getLandingAction();
            case "SetFCUModeAction":
                return actionProvider.getSetFCUModeAction(repr.newMode);
            case "PX4TakeoffAction":
                return actionProvider.getPX4TakeoffAction(repr.target_heightm);
            case "PX4LandAction":
                return actionProvider.getPx4LandAction();
            case "FlyToAction":
                List<Float> xyz1 = repr.waypoint;
                Waypoint objective1 = new Waypoint(xyz1.get(0), xyz1.get(1), xyz1.get(2));
                return actionProvider.getFlyToAction(objective1);
            case "PX4FlyToAction":
                seq++;
                List<Float> xyz2 = repr.waypoint;
                Waypoint objective2 = new Waypoint(xyz2.get(0), xyz2.get(1), xyz2.get(2));
                return actionProvider.getPX4FlyToAction(objective2,seq);
            default:
                log.warn("Flight io contained invalid action: " + repr.action);
                return null;
        }
    }

    private class JsonStateList {
        List<StateJsonRepresentation> queue;
    }

    private class StateJsonRepresentation {
        String state;
        List<ActionJsonRepresentation> scriptedActions;
        long duration;
        //other types of parameters go here by name
    }

    private class ActionJsonRepresentation {
        String action;
        double target_heightm;
        Float target_heightcm;
        List<Float> waypoint;
        String newMode; // new mode for SetFCUModeAction
        //possible parameters go here by name
    }
}
