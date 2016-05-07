package com.state_machine.core.script;

import com.google.gson.Gson;
import com.state_machine.core.actions.Action;
import com.state_machine.core.providers.ActionProvider;
import com.state_machine.core.providers.StateProvider;
import com.state_machine.core.states.State;
import org.apache.commons.logging.Log;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class FlightScript {

    private ActionProvider actions;
    private List<Action> start;

    public FlightScript(String filePath, ActionProvider actions, Log log){
        this.actions = actions;
        Gson gson = new Gson();
        Scanner scanner = null;
        StringBuilder json = new StringBuilder();
        try{
            scanner = new Scanner(filePath);
            while(scanner.hasNextLine()){
                json.append(scanner.nextLine() + "\n");
            }
            ScriptStruct script = gson.fromJson(json.toString(), ScriptStruct.class);
            this.start = parseStart(script);
        } catch (Exception e){
            log.warn("Could not read flight script at " + filePath, e);
        } finally {
            if(scanner != null) scanner.close();
        }
    }

    public List<Action> getActions(){
        return start;
    }

    private List<Action> parseStart(ScriptStruct script){
        List<Action> list = new ArrayList<Action>();
        for(ScriptObject o : script.start){
            list.add(parseAction(o.action));
        }
        return list;
    }

    private Action parseAction(String name){
        switch(name){
            case "ArmAction":
                return actions.getArmAction();
            case "DisarmAction":
                return actions.getDisarmAction();
            case "TakeoffAction":
                return actions.getTakeoffAction();
            case "LandingAction":
                return actions.getLandingAction();
            default:
                throw new RuntimeException("Incorrect action in json: " + name);
        }
    }

    private class ScriptStruct{
        public List<ScriptObject> start;
        public ScriptObject repeat;
        public ScriptObject finish;
    }

    private class ScriptObject{
        public String action;
    }
}
