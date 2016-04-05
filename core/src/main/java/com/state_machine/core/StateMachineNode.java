package com.state_machine.core;

import org.apache.commons.logging.Log;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.HashMap;
import java.util.Map;

public class StateMachineNode extends AbstractNodeMain {

    private StateMachine stateMachine;
    private Map<String, State> states;

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rosjava/listener");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        states = initializeStates(connectedNode);
        stateMachine = new StateMachine(states.get("Arm"), states.get("Arm"));
        final Log log = connectedNode.getLog();

        connectedNode.executeCancellableLoop(new CancellableLoop() {
            //private int sequenceNumber;

            @Override
            protected void setup() {
                //sequenceNumber = 0;

            }

            @Override
            protected void loop() throws InterruptedException {

                //for reference
                //std_msgs.String str = publisher.newMessage();
                //str.setData("Hello world! " + sequenceNumber);
                //publisher.publish(str);



                //sequenceNumber++;
                Thread.sleep(1000);

            }
        });

    }

    public Map<String, State> initializeStates(ConnectedNode node) {
        Map<String, State> allStates = new HashMap<>();
        try {
            ServiceClient armingService = node.newServiceClient("mavros/cmd/arming", std_msgs.String._TYPE);
            allStates.put("Arm", new ArmState(armingService));
        }
        catch(ServiceNotFoundException ex){
            node.getLog().error("Service not found", ex);
        }
        return allStates;
    }
}
