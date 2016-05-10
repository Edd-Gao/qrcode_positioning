package com.state_machine.core.providers;

import com.state_machine.core.script.FlightScriptParser;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.List;
import java.util.Properties;

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

    public List<State> readScript(String filePath){
        return flightScriptParser.parseFile(filePath);
    }

    private void initConfig(){
        Properties properties = new Properties();
        InputStream input = null;
        try{
            input = new FileInputStream("config.properties");
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
