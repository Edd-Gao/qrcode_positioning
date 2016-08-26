package com.state_machine.core.providers;

import com.state_machine.core.io.FlightScriptParser;

import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.Queue;

import com.state_machine.core.states.State;
import org.apache.commons.logging.Log;

public class FileProvider {

    private Properties config;
    private FlightScriptParser flightScriptParser;

    public FileProvider(ActionProvider actionProvider, StateProvider stateProvider, Log log){
        flightScriptParser = new FlightScriptParser(actionProvider, stateProvider, log);
        initConfig();
    }

    public Properties getConfig(){ return config; }

    public Queue<State> readScript(String filePath){
        return flightScriptParser.parseFile(filePath);
    }

    private void initConfig(){
        Properties properties = new Properties();
        InputStream input = null;
        try{
            //todo: change the path format to relative path
            input = new FileInputStream("/home/firefly/workspace/DroneProject/drone-control/src/state_machine/core/src/main/java/com/state_machine/core/providers/config.properties");
            properties.load(input);
        } catch (Exception e){
            e.printStackTrace();
        } finally {
            try {
                if(input != null) input.close();
            } catch(IOException e) {
                e.printStackTrace();
            }
        }
        config = properties;
    }
}
