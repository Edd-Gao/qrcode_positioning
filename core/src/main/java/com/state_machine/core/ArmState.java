package com.state_machine.core;

import org.ros.node.service.ServiceClient;
import org.ros.node.topic.Publisher;

public class ArmState extends State {

    private ServiceClient armingService;

    public ArmState(ServiceClient armingService){
        this.armingService = armingService;
    }

    public void exitAction(){

    }

    public void entryAction(){
    }

    public void loopAction(){

    }
}
