package com.state_machine.core.providers;

import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import state_machine_msgs.*;

/**
 * Created by parallels on 8/18/16.
 */
public class RosServerProvider {

    private String currentAction;
    private boolean actionFinished;

    public RosServerProvider (ConnectedNode node){
        currentAction = "";
        actionFinished = false;

        node.newServiceServer("state_machine/actionStatus", ActionStatus._TYPE,
                new ServiceResponseBuilder<ActionStatusRequest, ActionStatusResponse>() {
                    @Override
                    public void build(ActionStatusRequest request, ActionStatusResponse response){
                        currentAction = request.getCurrentAction();
                        actionFinished = request.getActionFinished();

                        response.setSuccess(true);
                    }
                });
    }

    public String getCurrentAction(){return currentAction;}

    public boolean getActionFinished(){return actionFinished;}

    public void resetActionStatus(){
        currentAction = "";
        actionFinished = false;
    }


}
