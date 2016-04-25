package com.state_machine.core.providers;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

public class FileProvider {

    private Properties config;

    public FileProvider(){
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

    Properties getConfig(){ return config; }
}
